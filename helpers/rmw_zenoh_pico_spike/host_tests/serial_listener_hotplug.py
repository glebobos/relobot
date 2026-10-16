#!/usr/bin/env python3
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import time


def serial_descriptors(process_id):
    descriptors = {}
    for descriptor in Path(f"/proc/{process_id}/fd").glob("*"):
        try:
            target = os.readlink(descriptor)
        except FileNotFoundError:
            continue
        if target.startswith("/dev/pts/"):
            descriptors[descriptor.name] = target
    return descriptors


def wait_for(predicate, process, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"router exited early: {process.returncode}")
        if predicate():
            return True
        time.sleep(0.02)
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Isolated PTY hotplug during serial accept; run inside Docker."
    )
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument(
        "--router", default="/opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd"
    )
    arguments = parser.parse_args()
    evidence = arguments.evidence_dir.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    log_path = evidence / "router.log"
    device = evidence / "serial-device"
    config_path = evidence / "router.json"
    if log_path.exists() or config_path.exists() or device.is_symlink() or device.exists():
        parser.error("use a fresh evidence directory; earlier captures are not overwritten")
    started_utc = datetime.now(timezone.utc).isoformat()
    master_descriptors = []
    router = None
    try:
        with log_path.open("x") as log:
            original_master, original_slave = os.openpty()
            master_descriptors.append(original_master)
            original_target = os.ttyname(original_slave)
            device.symlink_to(original_target)
            os.close(original_slave)
            config_path.write_text(json.dumps({
                "mode": "router",
                "listen": {"endpoints": [f"serial/{device}#baudrate=115200"]},
                "scouting": {
                    "multicast": {"enabled": False},
                    "gossip": {"enabled": False},
                },
            }))
            environment = os.environ.copy()
            environment["ZENOH_ROUTER_CONFIG_URI"] = str(config_path)
            environment["RUST_LOG"] = "info,zenoh_link_serial=trace"
            router = subprocess.Popen(
                [arguments.router], env=environment,
                stdout=log, stderr=subprocess.STDOUT,
            )
            if not wait_for(
                lambda: original_target in serial_descriptors(router.pid).values()
                and "I'm cleaning the buffers" in log_path.read_text(),
                router, 10,
            ):
                raise RuntimeError("serial listener did not start; inspect router.log")
            time.sleep(0.2)
            before = serial_descriptors(router.pid)
            os.close(original_master)
            master_descriptors.remove(original_master)
            time.sleep(0.3)
            replacement_master, replacement_slave = os.openpty()
            master_descriptors.append(replacement_master)
            replacement_target = os.ttyname(replacement_slave)
            device.unlink()
            device.symlink_to(replacement_target)
            os.close(replacement_slave)
            reopened = wait_for(
                lambda: replacement_target in serial_descriptors(router.pid).values(),
                router, 5,
            )
            result = {
                "case": "hotplug_while_waiting_for_serial_handshake",
                "started_utc": started_utc,
                "observed_utc": datetime.now(timezone.utc).isoformat(),
                "original_target": original_target,
                "replacement_target": replacement_target,
                "descriptors_before": before,
                "descriptors_after": serial_descriptors(router.pid),
                "observation_window_seconds": 5,
                "reopened_replacement": reopened,
            }
            (evidence / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2), flush=True)
            print("PASS: replacement opened" if reopened
                  else "FAIL: replacement not opened; inspect retained descriptors",
                  flush=True)
            return 0 if reopened else 1
    finally:
        if router is not None and router.poll() is None:
            router.terminate()
            try:
                router.wait(timeout=3)
            except subprocess.TimeoutExpired:
                router.kill()
                router.wait(timeout=3)
        for descriptor in master_descriptors:
            os.close(descriptor)
        device.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
