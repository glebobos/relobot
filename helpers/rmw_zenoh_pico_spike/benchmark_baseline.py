#!/usr/bin/env python3
"""
Single-Core Pico-ROS Baseline Measurement Tool.

Measures:
- Command -> Ack roundtrip latency (min, mean, median, p95, p99, max, stddev)
- FreeRTOS dynamic heap: initial, end, min-ever under load
- FreeRTOS task stack watermarks: app_task, probe_task
- Local timeout detection latency upon command cessation
- Telemetry stability and publish error count
"""

import argparse
import json
import math
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


class BenchmarkNode(Node):
    def __init__(self, target_hz: float, duration_sec: float):
        super().__init__("picoros_baseline_benchmark")
        self.target_hz = target_hz
        self.duration_sec = duration_sec

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd = self.create_publisher(
            Float32, "/picoros_usb_spike/command", qos
        )

        self.latest_ack: Optional[float] = None
        self.latest_active: Optional[float] = None
        self.latest_timeouts: Optional[float] = None
        self.latest_heap_free: Optional[float] = None
        self.latest_heap_min: Optional[float] = None
        self.latest_stack_app: Optional[float] = None
        self.latest_stack_probe: Optional[float] = None
        self.latest_pub_errors: Optional[float] = None
        self.latest_uptime: Optional[float] = None

        self.pending_seq: Optional[int] = None
        self.send_timestamps: Dict[int, float] = {}
        self.rtt_measurements: List[float] = []

        self.create_subscription(Float32, "/picoros_usb_spike/ack", self._on_ack, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/active", self._on_active, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/timeouts", self._on_timeouts, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/heap_free", self._on_heap_free, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/heap_min_ever", self._on_heap_min, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/stack_app_words", self._on_stack_app, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/stack_probe_words", self._on_stack_probe, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/publish_errors", self._on_pub_errors, qos)
        self.create_subscription(Float32, "/picoros_usb_spike/value", self._on_uptime, qos)

    def _on_ack(self, msg: Float32):
        val = int(msg.data)
        self.latest_ack = val
        if val in self.send_timestamps:
            rtt_ms = (time.monotonic() - self.send_timestamps[val]) * 1000.0
            self.rtt_measurements.append(rtt_ms)
            del self.send_timestamps[val]

    def _on_active(self, msg: Float32):
        self.latest_active = msg.data

    def _on_timeouts(self, msg: Float32):
        self.latest_timeouts = msg.data

    def _on_heap_free(self, msg: Float32):
        self.latest_heap_free = msg.data

    def _on_heap_min(self, msg: Float32):
        self.latest_heap_min = msg.data

    def _on_stack_app(self, msg: Float32):
        self.latest_stack_app = msg.data

    def _on_stack_probe(self, msg: Float32):
        self.latest_stack_probe = msg.data

    def _on_pub_errors(self, msg: Float32):
        self.latest_pub_errors = msg.data

    def _on_uptime(self, msg: Float32):
        self.latest_uptime = msg.data


def spin_until(node: BenchmarkNode, condition_fn, timeout_sec: float = 10.0) -> bool:
    start = time.monotonic()
    while time.monotonic() - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition_fn():
            return True
    return False


def run_benchmark(target_hz: float, duration_sec: float) -> dict:
    rclpy.init()
    node = BenchmarkNode(target_hz, duration_sec)

    print(f"[*] Waiting for initial telemetry from Pico-ROS...")
    def ready():
        return (
            node.latest_ack is not None
            and node.latest_heap_free is not None
            and node.latest_heap_min is not None
            and node.latest_stack_app is not None
            and node.latest_stack_probe is not None
            and node.latest_timeouts is not None
        )

    if not spin_until(node, ready, timeout_sec=15.0):
        print("FAIL: Timeout waiting for initial telemetry topics.", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    init_ack = node.latest_ack
    init_heap_free = node.latest_heap_free
    init_heap_min = node.latest_heap_min
    init_stack_app = node.latest_stack_app
    init_stack_probe = node.latest_stack_probe
    init_timeouts = node.latest_timeouts
    init_pub_errors = node.latest_pub_errors
    init_uptime = node.latest_uptime

    print(f"[*] Initial state: Ack={init_ack}, Uptime={init_uptime:.1f}s, "
          f"HeapFree={init_heap_free:.0f}B, HeapMinEver={init_heap_min:.0f}B, "
          f"StackAppWords={init_stack_app:.0f}, StackProbeWords={init_stack_probe:.0f}")

    # Wait for publisher matching subscriber count == 1
    start_wait = time.monotonic()
    while node.pub_cmd.get_subscription_count() < 1:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - start_wait > 10.0:
            print("FAIL: Timeout waiting for subscriber matching /picoros_usb_spike/command", file=sys.stderr)
            rclpy.shutdown()
            sys.exit(1)

    total_cmds = int(target_hz * duration_sec)
    period = 1.0 / target_hz
    start_seq = int(init_ack) + 1
    sent_count = 0

    print(f"[*] Starting benchmark: {total_cmds} commands at {target_hz} Hz ({duration_sec}s duration)...")
    bench_start = time.monotonic()
    next_send_time = bench_start

    for i in range(total_cmds):
        seq = start_seq + i
        msg = Float32(data=float(seq))
        node.send_timestamps[seq] = time.monotonic()
        node.pub_cmd.publish(msg)
        sent_count += 1

        next_send_time += period
        while True:
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            if now >= next_send_time:
                break

    bench_end = time.monotonic()
    print(f"[*] Commands completed in {bench_end - bench_start:.2f}s. Sent: {sent_count}.")

    # Drain remaining acks for up to 3s
    drain_deadline = time.monotonic() + 3.0
    while time.monotonic() < drain_deadline and len(node.send_timestamps) > 0:
        rclpy.spin_once(node, timeout_sec=0.05)

    acked_count = len(node.rtt_measurements)
    last_cmd_time = bench_end

    print(f"[*] Measuring local timeout behavior (command timeout = 1000ms)...")
    timeout_detected_time = None
    timeout_deadline = time.monotonic() + 5.0
    while time.monotonic() < timeout_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest_active == 0.0 and node.latest_timeouts > init_timeouts:
            if timeout_detected_time is None:
                timeout_detected_time = time.monotonic()
                break

    # Read final state
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    end_heap_free = node.latest_heap_free
    end_heap_min = node.latest_heap_min
    end_stack_app = node.latest_stack_app
    end_stack_probe = node.latest_stack_probe
    end_pub_errors = node.latest_pub_errors
    end_timeouts = node.latest_timeouts
    end_uptime = node.latest_uptime

    rclpy.shutdown()

    # Calculate statistics
    rtts = sorted(node.rtt_measurements)
    rtt_min = rtts[0] if rtts else 0.0
    rtt_max = rtts[-1] if rtts else 0.0
    rtt_mean = sum(rtts) / len(rtts) if rtts else 0.0
    rtt_median = rtts[len(rtts) // 2] if rtts else 0.0
    p95 = rtts[int(len(rtts) * 0.95)] if rtts else 0.0
    p99 = rtts[int(len(rtts) * 0.99)] if rtts else 0.0
    variance = sum((x - rtt_mean) ** 2 for x in rtts) / len(rtts) if rtts else 0.0
    rtt_stddev = math.sqrt(variance)

    timeout_duration = (timeout_detected_time - last_cmd_time) if timeout_detected_time else None

    # FreeRTOS total heap is 128 KiB = 131072 bytes
    total_heap = 128 * 1024
    app_stack_alloc = 8192 * 4  # bytes
    probe_stack_alloc = 512 * 4  # bytes

    results = {
        "params": {
            "target_hz": target_hz,
            "duration_sec": duration_sec,
            "total_commands_planned": total_cmds,
            "total_commands_sent": sent_count,
            "total_commands_acked": acked_count,
            "ack_loss_rate_pct": ((sent_count - acked_count) / sent_count * 100.0) if sent_count else 0.0,
        },
        "latency_ms": {
            "min": round(rtt_min, 2),
            "mean": round(rtt_mean, 2),
            "median": round(rtt_median, 2),
            "p95": round(p95, 2),
            "p99": round(p99, 2),
            "max": round(rtt_max, 2),
            "stddev": round(rtt_stddev, 2),
        },
        "memory_heap_bytes": {
            "total_allocated": total_heap,
            "initial_free": init_heap_free,
            "end_free": end_heap_free,
            "min_ever_free": end_heap_min,
            "peak_usage": total_heap - end_heap_min,
            "peak_usage_pct": round((total_heap - end_heap_min) / total_heap * 100.0, 2),
        },
        "task_stack_bytes": {
            "app_task": {
                "allocated": app_stack_alloc,
                "high_water_mark_words": end_stack_app,
                "headroom_bytes": int(end_stack_app * 4),
                "peak_used_bytes": app_stack_alloc - int(end_stack_app * 4),
                "peak_used_pct": round((app_stack_alloc - int(end_stack_app * 4)) / app_stack_alloc * 100.0, 2),
            },
            "probe_task": {
                "allocated": probe_stack_alloc,
                "high_water_mark_words": end_stack_probe,
                "headroom_bytes": int(end_stack_probe * 4),
                "peak_used_bytes": probe_stack_alloc - int(end_stack_probe * 4),
                "peak_used_pct": round((probe_stack_alloc - int(end_stack_probe * 4)) / probe_stack_alloc * 100.0, 2),
            },
        },
        "telemetry_and_safety": {
            "initial_uptime_sec": round(init_uptime, 2),
            "end_uptime_sec": round(end_uptime, 2),
            "publish_errors_delta": int(end_pub_errors - init_pub_errors),
            "timeouts_delta": int(end_timeouts - init_timeouts),
            "local_timeout_delay_sec": round(timeout_duration, 3) if timeout_duration else None,
        },
    }
    return results


def main():
    parser = argparse.ArgumentParser(description="Pico-ROS Single-Core Baseline Benchmark")
    parser.add_argument("--hz", type=float, default=10.0, help="Command publishing rate (default: 10 Hz)")
    parser.add_argument("--duration", type=float, default=30.0, help="Test duration in seconds (default: 30.0)")
    parser.add_argument("--json", action="store_true", help="Output raw JSON")
    args = parser.parse_args()

    results = run_benchmark(target_hz=args.hz, duration_sec=args.duration)

    if args.json:
        print(json.dumps(results, indent=2))
        return

    print("\n" + "=" * 60)
    print("   PICO-ROS SINGLE-CORE BASELINE RESULTS")
    print("=" * 60)
    p = results["params"]
    print(f"Sent / Acked:      {p['total_commands_sent']} / {p['total_commands_acked']} (loss: {p['ack_loss_rate_pct']}%)")
    lat = results["latency_ms"]
    print(f"Roundtrip Latency: min={lat['min']} ms, mean={lat['mean']} ms, median={lat['median']} ms, "
          f"p95={lat['p95']} ms, p99={lat['p99']} ms, max={lat['max']} ms (stddev={lat['stddev']} ms)")
    mem = results["memory_heap_bytes"]
    print(f"FreeRTOS Heap:     Total={mem['total_allocated']} B | MinEverFree={mem['min_ever_free']} B | PeakUsage={mem['peak_usage']} B ({mem['peak_usage_pct']}%)")
    stk = results["task_stack_bytes"]
    print(f"App Task Stack:    Alloc={stk['app_task']['allocated']} B | PeakUsed={stk['app_task']['peak_used_bytes']} B ({stk['app_task']['peak_used_pct']}%) | Headroom={stk['app_task']['headroom_bytes']} B")
    print(f"Probe Task Stack:  Alloc={stk['probe_task']['allocated']} B | PeakUsed={stk['probe_task']['peak_used_bytes']} B ({stk['probe_task']['peak_used_pct']}%) | Headroom={stk['probe_task']['headroom_bytes']} B")
    safe = results["telemetry_and_safety"]
    print(f"Telemetry Status:  PublishErrors={safe['publish_errors_delta']} | TimeoutsDelta={safe['timeouts_delta']} | TimeoutDelay={safe['local_timeout_delay_sec']} s")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
