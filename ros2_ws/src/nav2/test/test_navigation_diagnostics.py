import math

import pytest

from frontier_explorer.navigation_metrics import (
    OrderedPathTracker,
    motion_kind,
    observed_scan_clearance,
    point_footprint_distance,
    wheel_speeds,
)


def test_single_wheel_pivot_uses_half_track_radius():
    speeds = wheel_speeds(0.295 / 2.0, 1.0, 0.295, 0.0937)
    assert speeds['left_rad_s'] == pytest.approx(0.0)
    assert speeds['right_rad_s'] == pytest.approx(0.295 / 0.0937)
    speeds = wheel_speeds(0.0, 0.2, 0.295, 0.0937)
    assert speeds['left_rad_s'] == pytest.approx(-speeds['right_rad_s'])


@pytest.fixture
def offline_analyzer():
    from importlib.util import module_from_spec, spec_from_file_location
    from pathlib import Path

    helper = Path(__file__).resolve().parents[4] / 'helpers/navigation_diagnostics/analyze.py'
    spec = spec_from_file_location('navigation_analysis_helper', helper)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_offline_command_metrics_weight_time_and_exclude_stopping(offline_analyzer):
    rows = [(0.0, 0.0, 0.0), (1.0, .2, 0.0), (2.0, 0.0, 0.0),
            (2.6, .2, 0.0), (9.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    result = offline_analyzer.command_metrics(rows, 0, 10, .295, .0937, .2, .2)
    assert result['observed_settled_s'] == pytest.approx(8)
    assert result['below_deadband_s'] == pytest.approx(.6)
    assert result['below_deadband_fraction'] == pytest.approx(.075)
    assert result['longest_below_deadband_s'] == pytest.approx(.6)
    assert result['envelope_violation_count'] == 0
    rows[2] = (2.0, -.1, 0.0)
    assert offline_analyzer.command_metrics(rows, 0, 10, .295, .0937, .2, .2)[
        'envelope_violation_count'] == 1


def test_offline_command_metrics_distinguish_missing_data(offline_analyzer):
    assert offline_analyzer.command_metrics([], 0, 10, .295, .0937, .2, .2) == {'samples': 0}
    with pytest.raises(ValueError, match='backwards'):
        offline_analyzer.command_metrics([(2, .2, 0), (1, .2, 0)], 0, 10, .295, .0937, .2, .2)


@pytest.mark.parametrize('scenario', ['no_bag', 'multiple_paths', 'clock_reset', 'backwards', 'resume'])
def test_offline_analysis_bounds_capture_scope(offline_analyzer, tmp_path, scenario):
    import json

    samples = [{'ros_time_s': stamp, 'coverage_status': {'state': 'executing', 'progress_m': .2}}
               for stamp in (1.0, 2.0)]
    events = [{'ros_time_s': 0.0, 'event': 'plan_received', 'source': '/coverage/execution_path', 'poses': 2}]
    if scenario == 'multiple_paths':
        events.append(dict(events[0]))
    elif scenario == 'clock_reset':
        events.append({'ros_time_s': 1.0, 'event': 'clock_reset'})
    elif scenario == 'backwards':
        samples.reverse()
    elif scenario == 'resume':
        events.append({'ros_time_s': 1.5, 'event': 'coverage_status', 'status': {'state': 'blocked'}})
    (tmp_path / 'samples.jsonl').write_text(''.join(json.dumps(sample) + '\n' for sample in samples))
    (tmp_path / 'events.jsonl').write_text(''.join(json.dumps(event) + '\n' for event in events))
    (tmp_path / 'run.json').write_text(json.dumps({'run_id': 'fixture', 'image_id': 'fixture'}))
    (tmp_path / 'summary.json').write_text('{}')
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}
    if scenario == 'no_bag':
        report = offline_analyzer.analyze(tmp_path)
        assert 'unavailable' in report['tracking']
        assert report['observed_body_clearance_m'] == {'samples': 0}
        assert report['terminal_state'] == 'not_recorded'
    else:
        with pytest.raises(ValueError):
            offline_analyzer.analyze(tmp_path)
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == before


def test_offline_tracking_seeds_once_and_keeps_ordered_search(offline_analyzer):
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    from tf2_ros import Buffer

    path = Path()
    path.header.frame_id = 'map'
    for position_x in (0.0, 10.0):
        pose = PoseStamped()
        pose.pose.position.x = position_x
        path.poses.append(pose)
    buffer = Buffer()
    odometry = []
    for stamp, position_x in ((2, 5.0), (3, 8.0)):
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.header.stamp.sec = stamp
        transform.transform.translation.x = position_x
        transform.transform.rotation.w = 1.0
        buffer.set_transform(transform, 'test')
        message = Odometry()
        message.header.stamp.sec = stamp
        odometry.append((stamp, message))
    active = [{'ros_time_s': 1.0, 'coverage_status': {'progress_m': 4.9}},
              {'ros_time_s': 3.0, 'coverage_status': {'progress_m': 8.0}}]
    report = offline_analyzer.reconstruct_tracking(path, odometry, buffer, active, 1, 4)
    assert report['seed_progress_m'] == 4.9
    assert report['first_sample']['progress_m'] == pytest.approx(5)
    assert report['invalid_tracking_samples'] == 1
    assert report['last_sample']['progress_m'] == pytest.approx(5.6)
    assert report['remaining_at_last_sample_m'] == pytest.approx(5)
    with pytest.raises(ValueError, match='preceding manager progress'):
        offline_analyzer.reconstruct_tracking(path, odometry, buffer, [], 1, 4)


@pytest.mark.parametrize('linear,angular,expected', [
    (0.0, 0.0, 'stopped'), (0.0, 0.5, 'stationary_turn'),
    (0.2, 0.01, 'forward'), (-0.1, 0.2, 'reverse'),
    (math.nan, 0.0, 'invalid'),
])
def test_motion_classification(linear, angular, expected):
    assert motion_kind(linear, angular) == expected


def test_tracker_does_not_jump_to_a_nearby_return_swath():
    tracker = OrderedPathTracker([(0.0, 0.0), (4.0, 0.0), (4.0, 0.24), (0.0, 0.24)])
    result = tracker.update(0.5, 0.20, 0.0)
    assert result['segment'] == 0
    assert result['cross_track_m'] == pytest.approx(0.20)
    assert result['progress_m'] == pytest.approx(0.5)


def test_tracker_progress_and_heading_wrap():
    tracker = OrderedPathTracker([(0.0, 0.0), (0.0, 0.0), (0.0, 2.0)])
    result = tracker.update(0.1, 0.5, -3.0 * math.pi / 2.0)
    assert result['cross_track_m'] == pytest.approx(-0.1)
    assert result['heading_error_rad'] == pytest.approx(0.0)
    tracker.update(0.1, 0.45, 0.0)
    assert tracker.progress == pytest.approx(0.5)


def test_tracker_refuses_to_reacquire_after_a_large_pose_jump():
    tracker = OrderedPathTracker([(0.0, 0.0), (10.0, 0.0)])
    result = tracker.update(8.0, 0.0, 0.0)
    assert not result['tracking_valid']
    assert tracker.progress == 0.0
    assert result['path_distance_m'] == pytest.approx(7.0)


def test_tracker_handles_invalid_or_empty_paths():
    assert OrderedPathTracker([]).update(0.0, 0.0, 0.0) is None
    with pytest.raises(ValueError):
        OrderedPathTracker([(0.0, math.nan)])


@pytest.mark.parametrize('point,expected', [
    ((0.71, 0.0), 0.20), ((0.0, 0.375), 0.20),
    ((0.0, 0.0), 0.0), ((0.51, 0.175), 0.0),
])
def test_clearance_is_from_asymmetric_body_not_base_origin(point, expected):
    footprint = [(-0.10, -0.245), (-0.10, 0.175), (0.51, 0.175), (0.51, -0.245)]
    assert point_footprint_distance(point, footprint) == pytest.approx(expected)


@pytest.mark.parametrize('rotation,expected', [
    ((0.0, 0.0, 0.0, 1.0), 0.20),
    ((1.0, 0.0, 0.0, 0.0), 0.13),
    ((math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0), 0.015),
])
def test_planar_lidar_mounts_preserve_reflection_and_sensor_offset(rotation, expected):
    footprint = [(-0.10, -0.245), (-0.10, 0.175), (0.51, 0.175), (0.51, -0.245)]
    result = observed_scan_clearance([0.375], math.pi / 2.0, 0.0, 0.01, 10.0,
                                    (0.15, 0.0), rotation, footprint)
    assert result['observed_clearance_m'] == pytest.approx(expected)


def test_tilted_lidar_is_not_reported_as_planar_clearance():
    with pytest.raises(ValueError, match='nonplanar_scan_transform'):
        observed_scan_clearance([1.0], 0.0, 0.0, 0.01, 10.0, (0.0, 0.0),
                                (math.sin(0.2), 0.0, 0.0, math.cos(0.2)),
                                [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)])


def test_summary_excludes_invalid_tracking_and_paused_motion(tmp_path):
    import json
    from frontier_explorer.navigation_diagnostics import summarize_samples

    path = tmp_path / 'samples.jsonl'
    samples = [
        {'clock_state': 'advancing', 'tracking': {'tracking_valid': True, 'cross_track_m': -0.02},
         'scan': {'observed_clearance_m': 0.25}, 'odometry': {'motion': 'stationary_turn'}},
        {'clock_state': 'paused', 'tracking': {'tracking_valid': False, 'cross_track_m': 99.0},
         'scan': None, 'odometry': {'motion': 'stationary_turn'}},
    ]
    path.write_text(''.join(json.dumps(sample) + '\n' for sample in samples))
    summary = summarize_samples(path)
    assert summary['abs_cross_track_p95_m'] == pytest.approx(0.02)
    assert summary['minimum_observed_scan_clearance_m'] == pytest.approx(0.25)
    assert summary['measured_motion_samples']['stationary_turn'] == 1


def test_recorder_has_no_motion_publishers_and_handles_missing_data(tmp_path):
    import json
    import rclpy
    from frontier_explorer.navigation_diagnostics import NavigationDiagnostics

    rclpy.init()
    node = NavigationDiagnostics(tmp_path)
    try:
        assert all(publisher.topic_name not in {'/cmd_vel', '/cmd_vel_nav'}
                   for publisher in node.publishers)
        node.sample()
        node.finish('test')
        sample = json.loads((tmp_path / 'samples.jsonl').read_text())
        assert not sample['tracking']['tracking_valid']
        assert sample['scan']['observed_clearance_m'] is None
        assert sample['/cmd_vel'] is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('clock_response', ['True', 'False', '42', 'unavailable'])
def test_host_recorder_uses_controller_clock(tmp_path, monkeypatch, clock_response):
    from importlib.util import module_from_spec, spec_from_file_location
    import json
    from pathlib import Path
    from types import SimpleNamespace
    from unittest.mock import Mock

    helper = Path(__file__).resolve().parents[4] / 'helpers/navigation_diagnostics/record.py'
    spec = spec_from_file_location('navigation_record_helper', helper)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.setattr(module.sys, 'argv', ['record.py', '--label', 'clock-test', '--duration', '1'])

    def output(command, timeout=20):
        if 'inspect' in command:
            return json.dumps({'id': 'container', 'image': 'image', 'running': True,
                               'mounts': [{'Destination': '/ros2_ws', 'Source': str(tmp_path / 'ros2_ws')}]})
        if 'ros2 param get' in command[-1]:
            assert command[-1].endswith('use_sim_time --hide-type')
            return clock_response + '\n'
        return ''

    def spawn(*args, **kwargs):
        run_directory = next((tmp_path / 'ros2_ws/log/navigation').iterdir())
        (run_directory / 'summary.json').write_text('{}')
        return Mock(stdout=iter(()), stdin=Mock(), wait=Mock(return_value=0), poll=Mock(return_value=0))

    process = Mock(side_effect=spawn)
    monkeypatch.setattr(module, 'command_output', output)
    monkeypatch.setattr(module.subprocess, 'Popen', process)
    monkeypatch.setattr(module.subprocess, 'run', Mock(return_value=SimpleNamespace(stdout=b'')))
    result = module.main()
    if clock_response not in ('True', 'False'):
        assert result == 1
        process.assert_not_called()
        assert not (tmp_path / 'ros2_ws/log/navigation').exists()
        return
    assert result == 0
    process.assert_called_once()
    assert f'use_sim_time:={clock_response.lower()}' in process.call_args.args[0][-1]
    metadata_path = next((tmp_path / 'ros2_ws/log/navigation').glob('*/run.json'))
    assert json.loads(metadata_path.read_text())['controller_sim_time'] == (clock_response == 'True')


def test_clock_reset_discards_previous_path_and_commands(tmp_path, monkeypatch):
    from types import SimpleNamespace
    import json
    from geometry_msgs.msg import Twist
    import rclpy
    from frontier_explorer.navigation_diagnostics import NavigationDiagnostics

    rclpy.init()
    node = NavigationDiagnostics(tmp_path)
    current_time = [10_000_000_000]
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=current_time[0]))
    try:
        monkeypatch.setattr(node, 'get_clock', lambda: clock)
        node.receive('/cmd_vel', Twist())
        node.tracker = OrderedPathTracker([(0.0, 0.0), (1.0, 0.0)])
        node.sample()
        current_time[0] = 5_000_000_000
        node.sample()
        assert node.tracker is None
        assert '/cmd_vel' not in node.latest
        node.finish('test')
        samples = [json.loads(line) for line in (tmp_path / 'samples.jsonl').read_text().splitlines()]
        assert samples[-1]['clock_state'] == 'reset'
    finally:
        monkeypatch.undo()
        node.destroy_node()
        rclpy.shutdown()


def test_humble_parameter_snapshot_and_stale_odometry(tmp_path):
    import json
    import time
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from frontier_explorer.navigation_diagnostics import NavigationDiagnostics

    rclpy.init()
    node = NavigationDiagnostics(tmp_path)
    controller = rclpy.create_node('diff_drive_controller')
    controller.declare_parameter('wheel_separation', 0.295)
    controller.declare_parameter('wheel_radius', 0.0937)
    controller.declare_parameter('test_velocity_limits', [0.3, 0.0, 1.5])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(controller)
    try:
        deadline = time.monotonic() + 5.0
        while node.wheel_geometry is None and time.monotonic() < deadline:
            node.request_parameters()
            executor.spin_once(timeout_sec=0.02)
        assert node.wheel_geometry == (0.295, 0.0937)
        params = json.loads((tmp_path / 'parameters.json').read_text())
        assert params['/diff_drive_controller']['test_velocity_limits'] == [0.3, 0.0, 1.5]
        odometry = Odometry()
        odometry.header.stamp.sec = 1
        node.receive('/odometry/filtered', odometry)
        assert node.fresh('/odometry/filtered') is None
        odometry.header.stamp = node.get_clock().now().to_msg()
        assert node.fresh('/odometry/filtered') is odometry
        node.finish('test')
    finally:
        executor.shutdown()
        controller.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_stamped_tf_tracking_and_observed_scan_clearance(tmp_path):
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    import rclpy
    from sensor_msgs.msg import LaserScan
    from frontier_explorer.navigation_diagnostics import NavigationDiagnostics

    rclpy.init()
    node = NavigationDiagnostics(tmp_path)
    try:
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.header.stamp = stamp
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.1
        transform.transform.rotation.w = 1.0
        node.tf_buffer.set_transform(transform, 'test')
        path = Path()
        path.header.frame_id = 'map'
        for position_x in (0.0, 3.0):
            pose = PoseStamped()
            pose.pose.position.x = position_x
            path.poses.append(pose)
        node.on_plan(path)
        odometry = Odometry()
        odometry.header.stamp = stamp
        tracking = node.tracking_sample(odometry)
        assert tracking['tracking_valid']
        assert tracking['cross_track_m'] == pytest.approx(0.1)
        node.footprint = [(-0.1, -0.2), (0.51, -0.2), (0.51, 0.2), (-0.1, 0.2)]
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp = stamp
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [0.71, math.inf, math.nan]
        node.receive('/scan', scan)
        assert node.scan_sample()['observed_clearance_m'] == pytest.approx(0.2)
        node.finish('test')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_execution_path_takes_priority_over_planner_updates(tmp_path):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    import rclpy
    from frontier_explorer.navigation_diagnostics import NavigationDiagnostics

    rclpy.init()
    node = NavigationDiagnostics(tmp_path)
    try:
        path = Path()
        path.header.frame_id = 'map'
        for position_x in (0.0, 3.0):
            pose = PoseStamped()
            pose.pose.position.x = position_x
            path.poses.append(pose)
        node.on_plan(path, '/coverage/execution_path')
        tracker = node.tracker
        node.on_plan(path)
        assert node.tracker is tracker
        assert node.plan_source == '/coverage/execution_path'
        node.on_plan(Path(), '/coverage/execution_path')
        node.on_plan(path)
        assert node.plan_source == '/plan'
        assert node.tracker is not tracker
        node.finish('test')
    finally:
        node.destroy_node()
        rclpy.shutdown()