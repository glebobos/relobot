#!/usr/bin/env python3
import argparse
import json
import multiprocessing
import os
from pathlib import Path
import re
import time


def participant(role, duration, probe_id):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float32

    rclpy.init()
    node = rclpy.create_node(f'relobot_dds_{role}_{probe_id}')
    topic = f'/relobot_test/dds_coexistence/{probe_id}'
    samples = []
    sent = 0
    matched = 0
    try:
        if role == 'receiver':
            endpoint = node.create_subscription(
                Float32, topic, lambda message: samples.append(message.data), qos_profile_sensor_data)
        else:
            endpoint = node.create_publisher(Float32, topic, 10)

            def publish():
                nonlocal sent
                sent += 1
                endpoint.publish(Float32(data=float(sent)))

            node.create_timer(0.1, publish)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            peers = node.count_publishers(topic) if role == 'receiver' else node.count_subscribers(topic)
            matched = max(matched, peers)
        return {
            'role': role, 'received': len(samples), 'sent': sent, 'matched_peers': matched,
            'passed': matched > 0 and (role == 'sender' or len(samples) >= 10),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def worker(role, duration, probe_id, results):
    results.put(participant(role, duration, probe_id))


def main():
    parser = argparse.ArgumentParser(description='Independent DDS peers on test-only topics; run in Docker.')
    parser.add_argument('--domain-id', type=int, default=211)
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--probe-id', default=f'probe_{os.getpid()}')
    parser.add_argument('--role', choices=('sender', 'receiver'))
    arguments = parser.parse_args()
    if not Path('/.dockerenv').is_file():
        parser.error('run inside Docker, without physical devices')
    if not 200 <= arguments.domain_id <= 229:
        parser.error('use an unused test domain in 200..229, not the robot domain')
    if not 3.0 <= arguments.duration <= 60.0:
        parser.error('duration must be 3..60 seconds')
    if not re.fullmatch(r'[A-Za-z0-9_]{1,40}', arguments.probe_id):
        parser.error('probe-id must contain only 1..40 letters, numbers or underscores')
    os.environ['ROS_DOMAIN_ID'] = str(arguments.domain_id)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    if arguments.role:
        result = participant(arguments.role, arguments.duration, arguments.probe_id)
        print(json.dumps(result), flush=True)
        return 0 if result['passed'] else 1

    context = multiprocessing.get_context('spawn')
    results = context.Queue()
    processes = [context.Process(target=worker, args=(
        role, arguments.duration, arguments.probe_id, results)) for role in ('receiver', 'sender')]
    observations = []
    try:
        for process in processes:
            process.start()
        for process in processes:
            process.join(timeout=arguments.duration + 8)
        if all(process.exitcode == 0 for process in processes):
            observations = [results.get(timeout=2) for _ in processes]
        passed = len(observations) == 2 and all(result['passed'] for result in observations)
        print(json.dumps({
            'passed': passed, 'domain_id': arguments.domain_id, 'probe_id': arguments.probe_id,
            'results': observations, 'exits': [process.exitcode for process in processes],
            'physical_or_bridge_qualification': False,
        }, indent=2), flush=True)
        return 0 if passed else 1
    finally:
        for process in processes:
            if process.is_alive():
                process.kill()
                process.join(timeout=2)
        results.close()


if __name__ == '__main__':
    raise SystemExit(main())
