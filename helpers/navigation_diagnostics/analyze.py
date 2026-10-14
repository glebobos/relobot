#!/usr/bin/env python3
"""Read one recorded coverage execution and emit a derived JSON report to stdout."""

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import inspect
import json
import math
from pathlib import Path

import numpy as np

from frontier_explorer.navigation_metrics import OrderedPathTracker


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {'samples': 0}
    if not np.isfinite(values).all():
        raise ValueError('Non-finite metric values')
    return dict(samples=len(values), minimum=float(values.min()),
                p05=float(np.quantile(values, .05)), median=float(np.median(values)),
                p95=float(np.quantile(values, .95)), maximum=float(values.max()))


def command_metrics(rows, start, end, separation, radius, max_speed, min_radius):
    active = [row for row in rows if start <= row[0] < end]
    if not active:
        return {'samples': 0}
    if any(second[0] < first[0] for first, second in zip(active, active[1:])):
        raise ValueError('Command timestamps moved backwards')
    settled = [row for row in active if start + 1 < row[0] < end - 1]
    total = coast = episode = longest = 0.0
    for current, following in zip(active, active[1:]):
        duration = max(0.0, min(following[0], end - 1) - max(current[0], start + 1))
        if not duration:
            continue
        total += duration
        slower = (current[1] - separation * abs(current[2]) / 2) / radius
        if slower < .35:
            coast += duration
            episode += duration
            longest = max(longest, episode)
        else:
            episode = 0.0
    violations = [row for row in active if not all(math.isfinite(value) for value in row)
                  or row[1] < -1e-6 or row[1] > max_speed + 1e-6
                  or abs(row[2]) > row[1] / min_radius + 1e-6]
    return {
        'samples': len(active), 'first_ros_s': active[0][0], 'last_ros_s': active[-1][0],
        'base_speed_m_s': distribution([row[1] for row in active]),
        'settled_turn_speed_m_s': distribution([row[1] for row in settled if abs(row[2]) >= .15]),
        'settled_slower_wheel_rad_s': distribution([
            (row[1] - separation * abs(row[2]) / 2) / radius for row in settled]),
        'below_deadband_s': coast, 'below_deadband_fraction': coast / total if total else None,
        'longest_below_deadband_s': longest, 'observed_settled_s': total,
        'envelope_violation_count': len(violations), 'first_violations': violations[:5],
        'maximum_message_gap_s': max((second[0] - first[0] for first, second
                                     in zip(active, active[1:])), default=0.0),
    }


def straight_intervals(points):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    progress = np.r_[0.0, np.cumsum(lengths)]
    headings = np.arctan2(np.diff(points[:, 1]), np.diff(points[:, 0]))
    changes = np.arctan2(np.sin(np.diff(headings)), np.cos(np.diff(headings)))
    curvature = np.abs(changes) / np.maximum((lengths[1:] + lengths[:-1]) / 2, 1e-8)
    straight = np.zeros(len(lengths), dtype=bool)
    straight[1:-1] = (curvature[:-1] < .05) & (curvature[1:] < .05) & (lengths[1:-1] > 1e-6)
    intervals, beginning = [], None
    for index, enabled in enumerate(list(straight) + [False]):
        if enabled and beginning is None:
            beginning = index
        elif not enabled and beginning is not None:
            if progress[index] - progress[beginning] > 1.0:
                intervals.append((float(progress[beginning]), float(progress[index])))
            beginning = None
    return intervals


def read_bag(folder, cache_seconds):
    import rosbag2_py
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    wanted = {'/tf', '/tf_static', '/coverage/execution_path', '/odometry/filtered',
              '/cmd_vel', '/cmd_vel_nav', '/joint_states'}
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
             if item.name in wanted}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    data = defaultdict(list)
    buffer = Buffer(cache_time=Duration(seconds=cache_seconds))
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        message = deserialize_message(raw, types[topic])
        if topic in ('/tf', '/tf_static'):
            for transform in message.transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(transform, 'recording')
                else:
                    buffer.set_transform(transform, 'recording')
        else:
            data[topic].append((stamp / 1e9, message))
    return data, buffer


def reconstruct_tracking(path, odometry, buffer, active, start, end):
    from rclpy.time import Time
    from tf2_ros import TransformException

    if len(path.poses) < 2 or not path.header.frame_id:
        raise ValueError('Execution path requires a frame and at least two poses')
    points = np.array([(pose.pose.position.x, pose.pose.position.y) for pose in path.poses])
    tracker = OrderedPathTracker(points.tolist(), search_distance=.6)
    intervals = straight_intervals(points)
    sample_times = [sample['ros_time_s'] for sample in active]
    records, missing = [], Counter()
    seed = None
    previous_stamp = -math.inf
    for received, message in odometry:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if stamp < previous_stamp:
            raise ValueError('Odometry timestamps moved backwards')
        previous_stamp = stamp
        if not start <= stamp < end:
            continue
        try:
            transform = buffer.lookup_transform(path.header.frame_id, 'base_link',
                                                Time.from_msg(message.header.stamp)).transform
        except TransformException as error:
            missing[type(error).__name__] += 1
            continue
        if seed is None:
            index = bisect_right(sample_times, stamp) - 1
            status = active[index]['coverage_status'] if index >= 0 else {}
            seed = status.get('progress_m')
            if seed is None or not math.isfinite(seed) or not 0 <= seed <= tracker.length:
                raise ValueError('No valid preceding manager progress to seed ordered tracking')
            tracker.progress = seed
        rotation = transform.rotation
        yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
        tracking = tracker.update(transform.translation.x, transform.translation.y, yaw)
        if tracking is None:
            missing['invalid_pose_or_path'] += 1
            continue
        records.append(dict(tracking, ros_time_s=stamp,
                            straight=any(lower + .5 < tracking['progress_m'] < upper - .5
                                         for lower, upper in intervals),
                            linear=message.twist.twist.linear.x,
                            angular=message.twist.twist.angular.z))
    return {
        'seed_progress_m': seed, 'path_length_m': tracker.length,
        'excluded_samples': dict(missing),
        'invalid_tracking_samples': sum(not row['tracking_valid'] for row in records),
        'distance_m': distribution([row['path_distance_m'] for row in records]),
        'settled_straight_distance_m': distribution([row['path_distance_m'] for row in records if row['straight']]),
        'other_distance_m': distribution([row['path_distance_m'] for row in records if not row['straight']]),
        'measured_turn_speed_m_s': distribution([row['linear'] for row in records if abs(row['angular']) >= .15]),
        'remaining_at_last_sample_m': tracker.length - tracker.progress if records else None,
        'first_sample': records[0] if records else None,
        'last_sample': records[-1] if records else None,
        'straight_intervals_m': intervals,
    }


def analyze(folder):
    samples = [json.loads(line) for line in (folder / 'samples.jsonl').open()]
    events = [json.loads(line) for line in (folder / 'events.jsonl').open()]
    metadata = json.loads((folder / 'run.json').read_text())
    for records in (samples, events):
        timestamps = [record['ros_time_s'] for record in records]
        if any(not math.isfinite(stamp) for stamp in timestamps) or any(
                second < first for first, second in zip(timestamps, timestamps[1:])):
            raise ValueError('Recording timestamps must be finite and nondecreasing')
    paths = [event for event in events if event['event'] == 'plan_received'
             and event.get('source') == '/coverage/execution_path' and event.get('poses', 0) > 0]
    if len(paths) != 1 or any(event['event'] == 'clock_reset' for event in events):
        raise ValueError('Analysis requires one execution path and no clock reset; split multi-mission captures')
    active = [sample for sample in samples if (sample.get('coverage_status') or {}).get('state') == 'executing']
    if not active:
        raise ValueError('No recorded active coverage samples')
    start = active[0]['ros_time_s']
    terminal = next((event for event in events if event['event'] == 'coverage_status'
                     and event['ros_time_s'] >= start
                     and event['status']['state'] in ('completed', 'blocked', 'canceled', 'failed')), None)
    end = terminal['ros_time_s'] if terminal else active[-1]['ros_time_s']
    if end <= start or any(sample['ros_time_s'] > end for sample in active):
        raise ValueError('Analysis requires one positive-duration execution interval, without resume')
    parameters = {event['node']: event['values'] for event in events if event['event'] == 'parameter_snapshot'}
    report = {
        'schema_version': 1, 'run_id': metadata['run_id'], 'capture_image_id': metadata['image_id'],
        'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'tracker_sha256': hashlib.sha256(Path(inspect.getfile(OrderedPathTracker)).read_bytes()).hexdigest(),
        'observed_start_ros_s': start, 'observed_end_ros_s': end, 'observed_active_s': end - start,
        'terminal_state': terminal['status']['state'] if terminal else 'not_recorded',
        'initial_manager_progress_m': active[0]['coverage_status'].get('progress_m'),
        'original_summary': json.loads((folder / 'summary.json').read_text()),
        'observed_body_clearance_m': distribution([
            sample['scan']['observed_clearance_m'] for sample in active
            if (sample.get('scan') or {}).get('observed_clearance_m') is not None]),
        'limitations': [
            'Original summary is uncorrected evidence and may use the wrong segment after late attachment.',
            'Observed interval may omit startup, ingress or completion; not full mission duration.',
            'Tracking is seeded once from manager progress, not global nearest-row matching.',
            'Straight spans use curvature <0.05/m, length >1m, with 0.5m trimmed at both ends.',
            'Wheel gates use 0.35rad/s; settled excludes first/last second, turns use abs(w)>=0.15rad/s.',
            'Command durations hold values only between recorded messages; no extrapolation across bag boundaries.',
            'Scan clearance is observed only; joint states are not proof of loaded hardware behavior.',
        ],
    }
    if not (folder / 'bag' / 'metadata.yaml').exists():
        report['tracking'] = {'unavailable': 'No bag; original summary cannot be corrected'}
        return report
    data, buffer = read_bag(folder / 'bag', max(60.0, samples[-1]['ros_time_s'] - start + 60))
    paths = [message for stamp, message in data['/coverage/execution_path'] if message.poses]
    if len(paths) != 1:
        raise ValueError('Bag must contain exactly one nonempty execution path')
    report['tracking'] = reconstruct_tracking(paths[0], data['/odometry/filtered'], buffer, active, start, end)
    drive = parameters.get('/diff_drive_controller', {})
    controller = parameters.get('/controller_server', {})
    required = (drive.get('wheel_separation'), drive.get('wheel_radius'),
                controller.get('CoverageFollowPath.vx_max'),
                controller.get('CoverageFollowPath.AckermannConstraints.min_turning_r'))
    if any(value is None or not math.isfinite(value) or value <= 0 for value in required):
        report['commands'] = {'unavailable': 'Missing positive wheel geometry or coverage limits in snapshot'}
    else:
        report['commands'] = {topic: command_metrics(
            [(stamp, message.linear.x, message.angular.z) for stamp, message in data[topic]],
            start, end, *required) for topic in ('/cmd_vel', '/cmd_vel_nav')}
    names = drive.get('left_wheel_names', []) + drive.get('right_wheel_names', [])
    wheels = []
    if len(names) == 2:
        for stamp, message in data['/joint_states']:
            velocities = dict(zip(message.name, message.velocity))
            if start + 1 < stamp < end - 1 and all(name in velocities for name in names):
                wheels.append(min(velocities[name] for name in names))
    report['measured_slower_wheel_rad_s'] = distribution(wheels)
    report['measured_below_deadband_samples'] = sum(speed < .35 for speed in wheels)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path, help='Explicit run directory, never an implicit latest run')
    options = parser.parse_args()
    try:
        print(json.dumps(analyze(options.capture), indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f'Cannot analyze capture: {error}\n')


if __name__ == '__main__':
    main()