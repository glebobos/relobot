#!/usr/bin/env python3
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import select
import subprocess
import time
import traceback

from serial_listener_hotplug import serial_descriptors, wait_for


INIT_FRAME = bytes.fromhex("02 01 01 01 01 01 01 01 00")
ACK_FRAME = bytes.fromhex("02 03 01 01 01 01 01 01 00")
BAD_CRC_FRAME = bytes.fromhex("02 01 01 02 01 01 01 01 00")
CASES = (
    "silent",
    "handshake",
    "bad-crc",
    "absent-at-start",
    "hotplug",
    "partial-hotplug",
    "missing-path",
    "repeat-hotplug",
)


class SerialListener:
    def __init__(self, evidence, router_path):
        self.evidence = evidence
        self.router_path = router_path
        self.router = None
        self.log = None
        self.masters = set()
        self.master = None
        self.target = None
        self.slaves = set()
        self.device = evidence / "serial-device"
        self.log_path = evidence / "router.log"
        self.observations = []
        self.cleanup_forced = False
        self.shutdown_seconds = None

    def create_pty(self):
        master, slave = os.openpty()
        self.masters.add(master)
        self.slaves.add(slave)
        os.set_blocking(master, False)
        target = os.ttyname(slave)
        if not target.startswith("/dev/pts/"):
            raise RuntimeError(f"not an isolated PTY: {target}")
        return master, slave, target

    def close_slave(self, slave):
        os.close(slave)
        self.slaves.remove(slave)

    def start(self, initially_absent=False):
        self.evidence.mkdir(parents=True, exist_ok=False)
        self.master, slave, self.target = self.create_pty()
        if not initially_absent:
            self.device.symlink_to(self.target)
        self.close_slave(slave)
        config_path = self.evidence / "router.json"
        config_path.write_text(json.dumps({
            "mode": "router",
            "listen": {"endpoints": [f"serial/{self.device}#baudrate=115200"]},
            "scouting": {
                "multicast": {"enabled": False},
                "gossip": {"enabled": False},
            },
        }))
        environment = os.environ.copy()
        environment["ZENOH_ROUTER_CONFIG_URI"] = str(config_path)
        environment["RUST_LOG"] = "info,zenoh_link_serial=trace,z_serial=trace"
        self.log = self.log_path.open("x")
        self.router = subprocess.Popen(
            [self.router_path], env=environment, stdout=self.log,
            stderr=subprocess.STDOUT,
        )
        if initially_absent:
            self.require(
                lambda: "Hint: Is the serial cable connected?" in self.log_path.read_text(),
                "missing-path retry did not start", seconds=10,
            )
            self.observations.append({"initial_path_missing": True})
            self.device.symlink_to(self.target)
        self.require(
            lambda: self.target in serial_descriptors(self.router.pid).values()
            and self.accepts() >= 1,
            "listener did not start", seconds=10,
        )
        time.sleep(0.05)

    def accepts(self):
        return self.log_path.read_text().count("I'm cleaning the buffers")

    def require(self, predicate, message, seconds=5):
        if not wait_for(predicate, self.router, seconds):
            raise AssertionError(message)

    def handshake(self):
        os.write(self.master, INIT_FRAME)
        received = bytearray()

        def acknowledged():
            readable, _, _ = select.select([self.master], [], [], 0)
            if readable:
                received.extend(os.read(self.master, 4096))
            return ACK_FRAME in received

        self.require(acknowledged, "serial Init+Ack not received", seconds=2)
        self.observations.append({"serial_handshake_ack_hex": received.hex()})

    def corrupt_then_handshake(self):
        previous_accepts = self.accepts()
        os.write(self.master, BAD_CRC_FRAME)
        time.sleep(0.4)
        descriptors = serial_descriptors(self.router.pid)
        if list(descriptors.values()) != [self.target]:
            raise AssertionError(f"unexpected serial descriptors: {descriptors}")
        self.observations.append({
            "bad_crc_frame_hex": BAD_CRC_FRAME.hex(),
            "serial_descriptors": descriptors,
            "outer_accept_attempts_after_error": self.accepts() - previous_accepts,
        })
        self.handshake()

    def replace(self, gap_seconds=0, partial=False):
        previous_target = self.target
        previous_accepts = self.accepts()
        replacement_master, replacement_slave, replacement_target = self.create_pty()
        if replacement_target == previous_target:
            raise AssertionError("replacement did not change PTY identity")
        if partial:
            os.write(self.master, INIT_FRAME[:3])
            time.sleep(0.1)
        os.close(self.master)
        self.masters.remove(self.master)
        self.device.unlink()
        if gap_seconds:
            time.sleep(gap_seconds)
        self.device.symlink_to(replacement_target)
        self.close_slave(replacement_slave)
        self.master, self.target = replacement_master, replacement_target
        started = time.monotonic()
        reopened = wait_for(
            lambda: self.target in serial_descriptors(self.router.pid).values()
            and self.accepts() > previous_accepts,
            self.router, 5,
        )
        descriptors = serial_descriptors(self.router.pid)
        replacement_visible = self.device.resolve(strict=True) == Path(self.target)
        replacement_openable = None
        if not reopened:
            descriptor = os.open(self.device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                replacement_openable = os.readlink(f"/proc/self/fd/{descriptor}") == self.target
            finally:
                os.close(descriptor)
        self.observations.append({
            "original_target": previous_target,
            "replacement_target": self.target,
            "missing_path_seconds": gap_seconds,
            "partial_frame_before_loss": partial,
            "observation_seconds": time.monotonic() - started,
            "replacement_visible": replacement_visible,
            "replacement_fresh_open_succeeded_without_router_owner": replacement_openable,
            "reopened_replacement": reopened,
            "serial_descriptors": descriptors,
        })
        if not reopened:
            raise AssertionError("replacement not accepted within five seconds")
        if list(descriptors.values()) != [self.target]:
            raise AssertionError(f"stale or extra serial descriptors: {descriptors}")

    def stop(self):
        started = time.monotonic()
        if self.router is not None and self.router.poll() is None:
            self.router.terminate()
            try:
                self.router.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.cleanup_forced = True
                self.router.kill()
                self.router.wait(timeout=3)
        self.shutdown_seconds = time.monotonic() - started
        if self.log is not None:
            self.log.close()
        for descriptor in self.masters | self.slaves:
            os.close(descriptor)
        self.device.unlink(missing_ok=True)


def run_case(case, evidence, router_path, cycles):
    listener = SerialListener(evidence, router_path)
    result = {"case": case, "started_utc": datetime.now(timezone.utc).isoformat()}
    try:
        listener.start(initially_absent=case == "absent-at-start")
        if case == "silent":
            initial_descriptors = serial_descriptors(listener.router.pid)
            initial_accepts = listener.accepts()
            time.sleep(1)
            if listener.router.poll() is not None:
                raise AssertionError("router exited during silent handshake")
            if initial_descriptors != serial_descriptors(listener.router.pid):
                raise AssertionError("silent handshake replaced its descriptor")
            if listener.accepts() != initial_accepts:
                raise AssertionError("silent handshake unnecessarily retried accept")
            listener.observations.append({"idle_seconds": 1, "descriptor_unchanged": True})
        elif case in ("handshake", "absent-at-start"):
            listener.handshake()
        elif case == "bad-crc":
            listener.corrupt_then_handshake()
        else:
            for _ in range(cycles if case == "repeat-hotplug" else 1):
                listener.replace(
                    gap_seconds=1 if case == "missing-path" else 0,
                    partial=case == "partial-hotplug",
                )
            listener.handshake()
        result["passed"] = True
    except Exception:
        result["passed"] = False
        result["error"] = traceback.format_exc()
    finally:
        listener.stop()
        result["observations"] = listener.observations
        result["cleanup_forced"] = listener.cleanup_forced
        result["shutdown_seconds"] = listener.shutdown_seconds
        result["completed_utc"] = datetime.now(timezone.utc).isoformat()
        evidence.mkdir(parents=True, exist_ok=True)
        (evidence / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description="Docker-only PTY serial listener matrix; no physical device access.")
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--router", default="/opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd")
    parser.add_argument("--case", choices=("all",) + CASES, default="all")
    parser.add_argument("--cycles", type=int, default=10)
    arguments = parser.parse_args()
    if not Path("/.dockerenv").is_file():
        parser.error("run inside an isolated Docker container")
    if arguments.cycles < 1 or arguments.cycles > 100:
        parser.error("--cycles must be in 1..100")
    evidence = arguments.evidence_dir.resolve()
    if evidence.exists():
        parser.error("use a fresh evidence directory")
    evidence.mkdir(parents=True)
    cases = CASES if arguments.case == "all" else (arguments.case,)
    results = []
    for case in cases:
        result = run_case(case, evidence / case, arguments.router, arguments.cycles)
        results.append(result)
        print(f"{'PASS' if result['passed'] else 'FAIL'} {case}", flush=True)
        if not result["passed"]:
            print(result["error"], flush=True)
    summary = {"passed": all(result["passed"] for result in results), "results": results}
    (evidence / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
