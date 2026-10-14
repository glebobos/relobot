"""Bounded, passive ROS navigation recording. Never publishes velocity commands."""

from __future__ import annotations

import argparse
from array import array
from collections import Counter
import json
import math
from pathlib import Path as FilePath
import select
import shutil
import signal
import subprocess
import sys
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_explorer.navigation_metrics import (
    OrderedPathTracker, motion_kind, observed_scan_clearance, wheel_speeds,
)


BAG_TOPICS = [
    '/clock', '/tf', '/tf_static', '/map', '/map_updates', '/scan', '/parameter_events',
    '/cmd_vel_nav', '/cmd_vel', '/diff_drive_controller/cmd_vel_out',
    '/odometry/filtered', '/diff_drive_controller/odom',
    '/joint_states', '/robot_joint_states', '/robot_joint_commands',
    '/plan', '/received_global_plan', '/transformed_global_plan',
    '/lookahead_point', '/lookahead_collision_arc', '/behavior_tree_log',
    '/local_costmap/costmap', '/local_costmap/costmap_updates',
    '/local_costmap/published_footprint',
    '/global_costmap/costmap', '/global_costmap/costmap_updates',
    '/global_costmap/published_footprint',
    '/coverage/command', '/coverage/status', '/coverage/preview_path', '/coverage/execution_path',
    '/coverage/polygon_active', '/coverage/obstacles_active', '/explore/status',
] + [f'/{action}/_action/status' for action in (
    'navigate_to_pose', 'navigate_through_poses', 'follow_path', 'compute_coverage_path',
    'spin', 'backup',
)]

PARAMETER_NODES = (
    '/controller_server', '/planner_server', '/velocity_smoother', '/behavior_server',
    '/bt_navigator', '/coverage_server', '/coverage_manager', '/explore_node',
    '/local_costmap/local_costmap', '/global_costmap/global_costmap',
    '/diff_drive_controller',
)


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list, array)):
        return [finite_json(item) for item in value]
    return value


def write_json(path, value):
    path.write_text(json.dumps(finite_json(value), indent=2, allow_nan=False) + '\n')


def yaw_of(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
    )


def summarize_samples(path):
    errors = []
    clearance = []
    motion_samples = Counter()
    motion_entries = Counter()
    previous_kind = None
    sample_count = 0
    clock_states = Counter()
    with path.open() as stream:
        for line in stream:
            sample = json.loads(line)
            sample_count += 1
            clock_states[sample['clock_state']] += 1
            tracking = sample.get('tracking') or {}
            if tracking.get('tracking_valid'):
                errors.append(abs(tracking['cross_track_m']))
            scan = sample.get('scan') or {}
            if scan.get('observed_clearance_m') is not None:
                clearance.append(scan['observed_clearance_m'])
            kind = (sample.get('odometry') or {}).get('motion', 'unavailable')
            if sample['clock_state'] != 'advancing':
                kind = 'clock_not_advancing'
            motion_samples[kind] += 1
            if kind != previous_kind:
                motion_entries[kind] += 1
            previous_kind = kind
    errors.sort()
    return {
        'samples': sample_count,
        'clock_states': dict(clock_states),
        'measured_motion_samples': dict(motion_samples),
        'measured_motion_entries': dict(motion_entries),
        'valid_tracking_samples': len(errors),
        'abs_cross_track_p95_m': errors[math.ceil(0.95 * len(errors)) - 1] if errors else None,
        'abs_cross_track_max_m': max(errors) if errors else None,
        'minimum_observed_scan_clearance_m': min(clearance) if clearance else None,
        'limitations': [
            'Tracking uses the active execution path or /plan, not coverage completeness.',
            'Laser returns measure observed clearance only, not occluded space or ground truth.',
            'Motion entries are sampled transitions, not a diagnosis of their cause.',
        ],
    }


class NavigationDiagnostics(Node):
    def __init__(self, output: FilePath):
        super().__init__('navigation_diagnostics')
        self.output = output
        self.samples = (output / 'samples.jsonl').open('x', buffering=1)
        self.events = (output / 'events.jsonl').open('x', buffering=1)
        self.started = time.monotonic()
        self.latest = {}
        self.counts = Counter()
        self.parameters = {}
        self.parameter_clients = {
            name: (self.create_client(ListParameters, name + '/list_parameters'),
                   self.create_client(GetParameters, name + '/get_parameters'))
            for name in PARAMETER_NODES
        }
        self.pending_parameters = set()
        self.footprint = None
        self.wheel_geometry = None
        self.base_frame = 'base_link'
        self.tracker = None
        self.plan_frame = ''
        self.plan_source = '/plan'
        self.coverage_active = False
        self.plan_revision = 0
        self.previous_ros_time = None
        self.last_clock_advance = time.monotonic()
        self.last_states = {}
        self.scan_result = None
        self.scan_key = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for topic, message_type in (
            ('/cmd_vel_nav', Twist), ('/cmd_vel', Twist), ('/odometry/filtered', Odometry),
            ('/scan', LaserScan), ('/joint_states', JointState),
            ('/robot_joint_states', JointState), ('/robot_joint_commands', JointState),
        ):
            self.create_subscription(message_type, topic,
                                     lambda msg, name=topic: self.receive(name, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self.on_plan, qos_profile_sensor_data)
        self.create_subscription(Path, '/coverage/execution_path',
                     lambda msg: self.on_plan(msg, '/coverage/execution_path'),
                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/coverage/status', self.on_coverage, 10)
        self.create_subscription(String, '/coverage/command',
                                 lambda msg: self.event('coverage_command', command=msg.data), 10)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in (name for name in BAG_TOPICS if name.endswith('/_action/status')):
            self.create_subscription(GoalStatusArray, topic,
                                     lambda msg, name=topic: self.on_action(name, msg), status_qos)
        self.event('recorder_started', use_sim_time=self.get_parameter('use_sim_time').value)

    def event(self, kind, **fields):
        payload = {'event': kind, 'wall_elapsed_s': time.monotonic() - self.started,
                   'ros_time_s': self.get_clock().now().nanoseconds / 1e9, **fields}
        self.events.write(json.dumps(finite_json(payload), allow_nan=False) + '\n')

    def receive(self, topic, message):
        self.latest[topic] = (message, time.monotonic())
        self.counts[topic] += 1

    def on_plan(self, message, source='/plan'):
        self.receive(source, message)
        if source == '/plan' and self.coverage_active:
            return
        if source == '/coverage/execution_path':
            self.coverage_active = bool(message.poses)
        self.plan_source = source
        self.plan_revision += 1
        self.plan_frame = message.header.frame_id
        try:
            self.tracker = OrderedPathTracker([
                (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
            ])
        except ValueError as exc:
            self.tracker = None
            self.event('invalid_plan', reason=str(exc))
        self.event('plan_received', revision=self.plan_revision, frame=self.plan_frame,
                   source=source, poses=len(message.poses), length_m=self.tracker.length if self.tracker else None)

    def on_coverage(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            self.event('invalid_coverage_status', data=message.data)
            return
        if not isinstance(status, dict):
            return
        self.latest['coverage_status'] = (status, time.monotonic())
        identity = (status.get('state'), status.get('message'))
        if identity != self.last_states.get('coverage'):
            self.last_states['coverage'] = identity
            self.event('coverage_status', status=status)

    def on_action(self, topic, message):
        statuses = [(bytes(item.goal_info.goal_id.uuid).hex(), item.status)
                    for item in message.status_list]
        if statuses != self.last_states.get(topic):
            self.last_states[topic] = statuses
            self.event('action_status', topic=topic, goals=statuses)

    def request_parameters(self):
        for name, (list_client, get_client) in self.parameter_clients.items():
            if name in self.parameters or name in self.pending_parameters:
                continue
            if not list_client.service_is_ready() or not get_client.service_is_ready():
                continue
            self.pending_parameters.add(name)
            future = list_client.call_async(ListParameters.Request())
            future.add_done_callback(lambda result, node_name=name: self.on_parameter_names(node_name, result))

    def on_parameter_names(self, name, future):
        try:
            names = future.result().result.names
            request = self.parameter_clients[name][1].call_async(GetParameters.Request(names=names))
            request.add_done_callback(lambda result: self.on_parameters(name, names, result))
        except Exception as exc:
            self.pending_parameters.discard(name)
            self.event('parameter_snapshot_error', node=name, reason=str(exc))

    def on_parameters(self, name, names, future):
        self.pending_parameters.discard(name)
        try:
            values = future.result().values
            params = {key: parameter_value_to_python(value) for key, value in zip(names, values)}
            self.parameters[name] = params
            if name == '/local_costmap/local_costmap':
                footprint = json.loads(params.get('footprint', '[]'))
                if len(footprint) >= 3 and all(
                    len(point) == 2 and all(math.isfinite(float(value)) for value in point)
                    for point in footprint
                ):
                    self.footprint = [tuple(map(float, point)) for point in footprint]
                self.base_frame = params.get('robot_base_frame', 'base_link')
            if name == '/diff_drive_controller':
                separation = params.get('wheel_separation', 0.0)
                radius = params.get('wheel_radius', 0.0)
                if separation > 0.0 and radius > 0.0:
                    self.wheel_geometry = (separation, radius)
            write_json(self.output / 'parameters.json', self.parameters)
            self.event('parameter_snapshot', node=name, values=params)
        except Exception as exc:
            self.event('parameter_snapshot_error', node=name, reason=str(exc))

    def fresh(self, topic, max_age=0.5):
        entry = self.latest.get(topic)
        if entry is None or time.monotonic() - entry[1] > max_age:
            return None
        message = entry[0]
        if hasattr(message, 'header'):
            stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if stamp == 0.0 or age > max_age or age < -0.1:
                return None
        return message

    def twist_sample(self, twist):
        result = {'linear_m_s': twist.linear.x, 'angular_rad_s': twist.angular.z,
                  'motion': motion_kind(twist.linear.x, twist.angular.z)}
        if self.wheel_geometry:
            result['ideal_wheels'] = wheel_speeds(twist.linear.x, twist.angular.z, *self.wheel_geometry)
        return result

    def tracking_sample(self, odometry):
        if not self.tracker or not self.plan_frame or odometry is None:
            return {'tracking_valid': False, 'reason': 'missing_plan_or_fresh_odometry'}
        stamp = Time.from_msg(odometry.header.stamp)
        if stamp.nanoseconds == 0:
            return {'tracking_valid': False, 'reason': 'zero_odometry_stamp'}
        try:
            transform = self.tf_buffer.lookup_transform(self.plan_frame, self.base_frame, stamp).transform
            result = self.tracker.update(transform.translation.x, transform.translation.y,
                                         yaw_of(transform.rotation))
            return result or {'tracking_valid': False, 'reason': 'empty_path'}
        except TransformException as exc:
            return {'tracking_valid': False, 'reason': str(exc)}

    def scan_sample(self):
        scan = self.fresh('/scan')
        if scan is None or self.footprint is None:
            return {'observed_clearance_m': None, 'reason': 'missing_fresh_scan_or_footprint'}
        key = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if key == (0, 0):
            return {'observed_clearance_m': None, 'reason': 'zero_scan_stamp'}
        if key == self.scan_key:
            return self.scan_result
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, scan.header.frame_id, Time.from_msg(scan.header.stamp)).transform
        except TransformException as exc:
            return {'observed_clearance_m': None, 'reason': str(exc)}
        rotation = transform.rotation
        try:
            self.scan_result = observed_scan_clearance(
                scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                (transform.translation.x, transform.translation.y),
                (rotation.x, rotation.y, rotation.z, rotation.w), self.footprint,
            )
        except ValueError as exc:
            return {'observed_clearance_m': None, 'reason': str(exc)}
        self.scan_key = key
        return self.scan_result

    def sample(self):
        now = time.monotonic()
        ros_time = self.get_clock().now().nanoseconds / 1e9
        clock_state = 'advancing'
        if self.previous_ros_time is not None and ros_time < self.previous_ros_time:
            self.latest.clear()
            self.tracker = None
            self.coverage_active = False
            self.scan_key = None
            self.tf_buffer.clear()
            self.last_clock_advance = now
            clock_state = 'reset'
            self.event('clock_reset', previous_ros_time_s=self.previous_ros_time)
        elif self.previous_ros_time is None or ros_time > self.previous_ros_time:
            self.last_clock_advance = now
        elif now - self.last_clock_advance > 0.5:
            clock_state = 'paused'
        if ros_time == 0.0 and self.get_parameter('use_sim_time').value:
            clock_state = 'waiting_for_clock'
        self.previous_ros_time = ros_time
        odometry = self.fresh('/odometry/filtered')
        result = {
            'wall_elapsed_s': now - self.started, 'ros_time_s': ros_time,
            'clock_state': clock_state, 'plan_revision': self.plan_revision,
            'plan_source': self.plan_source,
            'received_counts': dict(self.counts),
            'receipt_age_s': {name: now - received for name, (_, received) in self.latest.items()},
            'tracking': self.tracking_sample(odometry) if clock_state == 'advancing' else None,
            'scan': self.scan_sample() if clock_state == 'advancing' else None,
            'odometry': self.twist_sample(odometry.twist.twist) if odometry else None,
        }
        if odometry:
            result['odometry']['stamp_age_s'] = ros_time - Time.from_msg(odometry.header.stamp).nanoseconds / 1e9
        for topic in ('/cmd_vel_nav', '/cmd_vel'):
            message = self.fresh(topic)
            result[topic] = self.twist_sample(message) if message else None
        for topic in ('/joint_states', '/robot_joint_states', '/robot_joint_commands'):
            message = self.fresh(topic)
            result[topic] = dict(zip(message.name, message.velocity)) if message else None
        result['coverage_status'] = self.latest.get('coverage_status', (None,))[0]
        self.samples.write(json.dumps(finite_json(result), allow_nan=False) + '\n')

    def graph_snapshot(self):
        topics = dict(self.get_topic_names_and_types())
        endpoints = {}
        for topic in BAG_TOPICS:
            endpoints[topic] = [{
                'node': info.node_namespace.rstrip('/') + '/' + info.node_name,
                'reliability': info.qos_profile.reliability.name,
                'durability': info.qos_profile.durability.name,
            } for info in self.get_publishers_info_by_topic(topic)]
        write_json(self.output / 'graph.json', {'topics': topics, 'publishers': endpoints})

    def finish(self, reason):
        self.event('recorder_stopped', reason=reason)
        if rclpy.ok():
            self.graph_snapshot()
        self.samples.close()
        self.events.close()
        summary = summarize_samples(self.output / 'samples.jsonl')
        summary.update(stop_reason=reason, received_counts=dict(self.counts),
                       missing_parameter_nodes=sorted(set(PARAMETER_NODES) - self.parameters.keys()))
        write_json(self.output / 'summary.json', summary)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=FilePath, required=True)
    parser.add_argument('--duration', type=float, default=300.0)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--max-mib', type=int, default=512)
    parser.add_argument('--no-bag', action='store_true')
    parser.add_argument('--stop-on-stdin', action='store_true')
    options, ros_args = parser.parse_known_args(args)
    if not 0.0 < options.duration <= 3600.0 or not 1.0 <= options.rate <= 20.0 or options.max_mib < 16:
        parser.error('duration must be (0, 3600], rate [1, 20], and max-mib >= 16')
    options.output.mkdir(parents=True, exist_ok=True)
    if (options.output / 'samples.jsonl').exists() or (options.output / 'bag').exists():
        parser.error('output already contains a recording')
    rclpy.init(args=ros_args)
    node = NavigationDiagnostics(options.output)
    bag = None
    bag_log = None
    reason = 'duration_elapsed'
    deadline = time.monotonic() + options.duration
    next_sample = time.monotonic()
    next_housekeeping = time.monotonic()
    try:
        if not options.no_bag:
            bag_log = (options.output / 'rosbag.log').open('x')
            command = ['ros2', 'bag', 'record', '--include-hidden-topics',
                       '--max-cache-size', '16777216', '-o', str(options.output / 'bag')]
            if node.get_parameter('use_sim_time').value:
                command.append('--use-sim-time')
            bag = subprocess.Popen(command + BAG_TOPICS, stdout=bag_log, stderr=subprocess.STDOUT)
        print(f'Navigation recording active: {options.output}', flush=True)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if options.stop_on_stdin and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                reason = 'operator_stop'
                break
            if bag is not None and bag.poll() is not None:
                reason = f'bag_exited_{bag.returncode}'
                break
            if now >= next_sample:
                node.sample()
                next_sample = now + 1.0 / options.rate
            if now >= next_housekeeping:
                node.request_parameters()
                size = sum(path.stat().st_size for path in options.output.rglob('*') if path.is_file())
                if size >= options.max_mib * 1024 ** 2 or shutil.disk_usage(options.output).free < 256 * 1024 ** 2:
                    reason = 'storage_limit'
                    break
                next_housekeeping = now + 2.0
    except KeyboardInterrupt:
        reason = 'operator_stop'
    finally:
        if bag is not None and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:
                bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bag.kill()
                bag.wait()
                reason += '_bag_finalize_timeout'
        if bag_log:
            bag_log.close()
        node.finish(reason)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'Recording finished: {reason}. Summary: {options.output / "summary.json"}', flush=True)
    return 0 if reason in {'duration_elapsed', 'operator_stop'} else 1


if __name__ == '__main__':
    sys.exit(main())