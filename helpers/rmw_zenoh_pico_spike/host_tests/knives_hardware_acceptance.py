#!/usr/bin/env python3
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time
import traceback

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


def main():
    parser = argparse.ArgumentParser(description='Command/timeout checks for the no-actuator KAmod knives bench.')
    parser.add_argument('--confirm-bench', action='store_true', required=True)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--report', type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.duration < 10:
        parser.error('use at least 10 seconds of continuous command load')
    result = {'passed': False, 'bench_only': True, 'duration_s': arguments.duration,
              'recovery_or_motor_qualification': False}
    samples = defaultdict(list)
    load_messages = defaultdict(int)
    rclpy.init()
    node = rclpy.create_node('knives_bench_acceptance')
    publisher = node.create_publisher(Float32, '/knives/set_rpm', 10)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    names = ['target_rpm', 'heap_free_bytes', 'heap_min_bytes', 'app_stack_free_words',
             'control_stack_free_words', 'publish_errors']
    subscriptions = []

    def record(name, message):
        value = float(message.data)
        if not math.isfinite(value):
            raise AssertionError(f'non-finite telemetry on {name}: {value!r}')
        samples[name].append((time.monotonic(), value))

    for name in names + ['current_rpm']:
        topic = '/knives/current_rpm' if name == 'current_rpm' else f'/knives/bench/{name}'
        subscriptions.append(node.create_subscription(
            Float32, topic, lambda message, key=name: record(key, message), qos))

    def count_load(name, message):
        load_messages[name] += 1

    subscriptions.append(node.create_subscription(Clock, '/clock', lambda message: count_load('clock', message), qos))
    subscriptions.append(node.create_subscription(LaserScan, '/scan', lambda message: count_load('scan', message), qos))

    def spin_for(seconds, value=None):
        deadline = time.monotonic() + seconds
        next_publish = time.monotonic()
        last_published = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if value is not None and now >= next_publish:
                publisher.publish(Float32(data=float(value)))
                last_published = now
                next_publish = now + 0.1
            rclpy.spin_once(node, timeout_sec=0.02)
        return last_published

    def command_phase(value, expected, seconds=3.0):
        started = time.monotonic()
        last_published = spin_for(seconds, value)
        observed = [sample for sample in samples['target_rpm'] if sample[0] >= started]
        assert any(abs(sample[1] - expected) < 0.01 for sample in observed), (
            f'command {value} did not produce target {expected}: {observed}')
        assert abs(observed[-1][1] - expected) < 0.01, f'target did not remain at {expected}'
        print(f'command {value} -> target {expected}: PASS', flush=True)
        return last_published

    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if publisher.get_subscription_count() and all(samples[name] for name in names):
                break
        else:
            raise AssertionError(f'firmware discovery/diagnostics incomplete: {list(samples)}')
        assert 'knives_mock' not in node.get_node_names(), 'Gazebo mock is still active'
        rpm_publishers = node.get_publishers_info_by_topic('/knives/current_rpm')
        command_subscribers = node.get_subscriptions_info_by_topic('/knives/set_rpm')
        assert len(rpm_publishers) == 1 and rpm_publishers[0].node_name == 'knives_node', (
            'RPM telemetry must come only from knives_node')
        assert len(command_subscribers) == 1 and command_subscribers[0].node_name == 'knives_node', (
            'commands must have exactly one firmware subscriber')
        assert node.count_publishers('/knives/set_rpm') == 1, 'another command source is active'
        command_phase(0.0, 0.0)
        started = time.monotonic()
        last_command = command_phase(1200.0, 1200.0, arguments.duration)
        ended = time.monotonic()
        rpm_samples = [sample for sample in samples['current_rpm'] if started <= sample[0] <= ended]
        rate = len(rpm_samples) / (ended - started)
        assert 8.0 <= rate <= 12.0, f'RPM telemetry rate {rate:.2f} Hz outside 8–12 Hz'
        assert all(abs(sample[1]) < 0.01 for sample in rpm_samples), 'unexpected encoder RPM on bench'
        assert load_messages['clock'] > 0 and load_messages['scan'] > 0, 'Gazebo clock/scan load missing'
        result['rpm_rate_hz'] = rate
        result['rpm_messages_during_load'] = len(rpm_samples)
        result['max_rpm_gap_s'] = max(
            later[0] - earlier[0] for earlier, later in zip(rpm_samples, rpm_samples[1:]))
        spin_for(7.5)
        zeros = [stamp for stamp, value in samples['target_rpm'] if stamp > last_command and value == 0.0]
        assert zeros, 'target did not expire after commands stopped'
        timeout_delay = zeros[0] - last_command
        assert 4.5 <= timeout_delay <= 7.5, f'command timeout observed after {timeout_delay:.3f}s'
        result['timeout_observed_after_s'] = timeout_delay
        print(f'command timeout observed after {timeout_delay:.3f}s: PASS', flush=True)
        command_phase(1200.0, 0.0)
        command_phase(0.0, 0.0)
        command_phase(-500.0, -500.0)
        command_phase(5000.0, 3500.0)
        command_phase(-5000.0, -3500.0)
        command_phase(0.0, 0.0)
        last_valid = command_phase(600.0, 600.0)
        invalid_started = time.monotonic()
        invalid_values = (math.nan, math.inf, -math.inf)
        for index in range(80):
            spin_for(0.1, invalid_values[index % len(invalid_values)])
        after_invalid = [sample for sample in samples['target_rpm'] if sample[0] >= invalid_started]
        assert after_invalid and all(value in (0.0, 600.0) for _, value in after_invalid), (
            f'invalid command changed target: {after_invalid}')
        expired = [stamp for stamp, value in after_invalid if value == 0.0]
        assert expired, 'NaN/Inf commands refreshed the command timeout'
        invalid_timeout = expired[0] - last_valid
        assert 4.5 <= invalid_timeout <= 7.5, f'invalid-command timeout observed after {invalid_timeout:.3f}s'
        assert after_invalid[-1][1] == 0.0
        result['invalid_commands_timeout_after_s'] = invalid_timeout
        result['resources'] = {}
        for name in names[1:]:
            values = [value for _, value in samples[name]]
            assert values, f'no diagnostic samples: {name}'
            result['resources'][name] = {'min': min(values), 'max': max(values), 'last': values[-1]}
            if name != 'publish_errors':
                assert min(values) > 0, f'no remaining resource headroom: {name}'
        assert result['resources']['publish_errors']['max'] == 0.0, 'firmware reported publication errors'
        result['passed'] = True
    except Exception:
        result['error'] = traceback.format_exc()
    finally:
        try:
            spin_for(0.4, 0.0)
        except Exception:
            result['cleanup_error'] = traceback.format_exc()
            result['passed'] = False
        result['load_messages'] = dict(load_messages)
        result['sample_counts'] = {name: len(values) for name, values in samples.items()}
        node.destroy_node()
        rclpy.shutdown()
        arguments.report.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
