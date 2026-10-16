#!/usr/bin/env python3
import argparse
import errno
import json
import os
from pathlib import Path
import select
import subprocess
import threading
import time
import traceback

from serial_listener_hotplug import serial_descriptors, wait_for


class SerialRelay:
    def __init__(self, masters):
        self.masters = masters
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.error = None
        self.transferred = {master: 0 for master in masters}

    def run(self):
        pending = {master: bytearray() for master in self.masters}
        peers = dict(zip(self.masters, reversed(self.masters)))
        try:
            while not self.stopped.is_set():
                readable, writable, _ = select.select(
                    self.masters, [master for master in self.masters if pending[master]], [], 0.02,
                )
                for master in readable:
                    try:
                        received = os.read(master, 4096)
                    except OSError as error:
                        if error.errno not in (errno.EIO, errno.EAGAIN):
                            raise
                        continue
                    pending[peers[master]].extend(received)
                    if len(pending[peers[master]]) > 1048576:
                        raise RuntimeError("virtual serial relay queue exceeded one MiB")
                for master in writable:
                    try:
                        written = os.write(master, pending[master])
                    except OSError as error:
                        if error.errno not in (errno.EIO, errno.EAGAIN):
                            raise
                        continue
                    del pending[master][:written]
                    self.transferred[master] += written
                if readable and not writable:
                    self.stopped.wait(0.005)
        except Exception:
            self.error = traceback.format_exc()


def cli_environment(config):
    environment = os.environ.copy()
    environment["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    environment["ZENOH_SESSION_CONFIG_URI"] = str(config)
    environment["RUST_LOG"] = "warn"
    return environment


def exchange(evidence, sender, receiver, value):
    topic = f"/serial_router_smoke_{value}"
    subscriber_log = evidence / f"subscriber-{value}.log"
    publisher_log = evidence / f"publisher-{value}.log"
    subscriber = None
    try:
        with subscriber_log.open("x") as output:
            subscriber = subprocess.Popen(
                ["timeout", "--kill-after=3s", "20s", "ros2", "topic", "echo", topic,
                 "std_msgs/msg/Float32", "--once", "--no-lost-messages",
                 "--filter", f"m.data == {value}.0", "--qos-reliability", "reliable",
                 "--qos-durability", "volatile"],
                env=cli_environment(receiver), stdout=output, stderr=subprocess.STDOUT,
            )
            with publisher_log.open("x") as published:
                result = subprocess.run(
                    ["timeout", "--kill-after=3s", "20s", "ros2", "topic", "pub", topic,
                     "std_msgs/msg/Float32", f"{{data: {value}.0}}", "--once",
                     "--wait-matching-subscriptions", "1", "--max-wait-time-secs", "10",
                     "--qos-reliability", "reliable", "--qos-durability", "volatile"],
                    env=cli_environment(sender), stdout=published, stderr=subprocess.STDOUT,
                    timeout=25,
                )
            if result.returncode != 0:
                raise AssertionError(f"publisher {value} failed: {result.returncode}")
            if subscriber.wait(timeout=25) != 0:
                raise AssertionError(f"subscriber {value} failed: {subscriber.returncode}")
        if f"data: {value}.0" not in subscriber_log.read_text().splitlines():
            raise AssertionError(f"typed value {value}.0 not received")
    finally:
        if subscriber is not None and subscriber.poll() is None:
            subscriber.terminate()
            try:
                subscriber.wait(timeout=4)
            except subprocess.TimeoutExpired:
                subscriber.kill()
                subscriber.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser(description="Stock ROS CLI across two routers connected only by relayed PTYs.")
    parser.add_argument("--evidence-dir", type=Path, required=True)
    arguments = parser.parse_args()
    if not Path("/.dockerenv").is_file():
        parser.error("run inside a network-isolated Docker container")
    evidence = arguments.evidence_dir.resolve()
    if evidence.exists():
        parser.error("use a fresh evidence directory")
    evidence.mkdir(parents=True)
    masters = []
    slaves = []
    targets = []
    routers = []
    logs = []
    relay = None
    result = {"passed": False, "forced_router_cleanup": []}
    try:
        for _ in range(2):
            master, slave = os.openpty()
            masters.append(master)
            slaves.append(slave)
            targets.append(os.ttyname(slave))
            os.set_blocking(master, False)
        if any(not target.startswith("/dev/pts/") for target in targets):
            raise RuntimeError("only container-local PTYs are allowed")
        for slave in slaves:
            os.close(slave)
        slaves.clear()
        relay = SerialRelay(masters)
        relay.thread.start()
        sessions = []
        for index, port in enumerate((17447, 17448)):
            endpoint = f"serial/{targets[index]}#baudrate=115200"
            config = {
                "mode": "router",
                "listen": {"endpoints": [f"tcp/127.0.0.1:{port}"]},
                "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
            }
            if index == 0:
                config["listen"]["endpoints"].append(endpoint)
            else:
                config["connect"] = {"endpoints": [endpoint]}
            config_path = evidence / f"router-{index}.json"
            config_path.write_text(json.dumps(config))
            session_path = evidence / f"session-{index}.json"
            session_path.write_text(json.dumps({
                "mode": "client", "connect": {"endpoints": [f"tcp/127.0.0.1:{port}"]},
                "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
            }))
            sessions.append(session_path)
            environment = os.environ.copy()
            environment["ZENOH_ROUTER_CONFIG_URI"] = str(config_path)
            environment["RUST_LOG"] = "info,zenoh_link_serial=trace,zenoh_transport=debug"
            log_path = evidence / f"router-{index}.log"
            output = log_path.open("x")
            logs.append(output)
            router = subprocess.Popen(
                ["/opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd"],
                env=environment, stdout=output, stderr=subprocess.STDOUT,
            )
            routers.append(router)
            if index == 0 and not wait_for(
                lambda: targets[0] in serial_descriptors(router.pid).values()
                and "I'm cleaning the buffers" in log_path.read_text(), router, 10,
            ):
                raise AssertionError("serial listener did not open")
        exchange(evidence, sessions[0], sessions[1], 37)
        exchange(evidence, sessions[1], sessions[0], 38)
        if relay.error is not None:
            raise RuntimeError(relay.error)
        if not all(relay.transferred.values()):
            raise AssertionError("serial relay did not transfer bytes both ways")
        result["passed"] = True
        result["typed_values_received"] = [37.0, 38.0]
        result["serial_targets"] = targets
    except Exception:
        result["error"] = traceback.format_exc()
    finally:
        for router in routers:
            if router.poll() is None:
                router.terminate()
        for index, router in enumerate(routers):
            try:
                router.wait(timeout=3)
            except subprocess.TimeoutExpired:
                result["forced_router_cleanup"].append(index)
                router.kill()
                router.wait(timeout=3)
        if relay is not None:
            relay.stopped.set()
            relay.thread.join(timeout=2)
            result["serial_bytes_written"] = list(relay.transferred.values())
            result["relay_error"] = relay.error
        for descriptor in masters + slaves:
            os.close(descriptor)
        for output in logs:
            output.close()
        (evidence / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
