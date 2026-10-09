from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import signal
import subprocess
import time
import math
import os

from action_msgs.msg import GoalStatus
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import pytest
import yaml

from frontier_explorer.coverage_execution import CoverageExecution


def await_future(node, future, timeout=10.0):
    import rclpy
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert future.done(), 'ROS request timed out'
    return future.result()


@pytest.mark.parametrize('phase', ['following', 'backing_up', 'navigating_ingress'])
def test_pending_cancel_is_sent_when_goal_is_accepted(phase):
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = True
    executor.handle = None
    executor.stop_state = None
    executor.cancel_sent = False
    executor.request_id = 1
    executor.phase = phase
    executor.report = Mock()
    executor.plan_pub = Mock()
    executor.cancel()
    assert executor.busy
    handle = Mock(accepted=True)
    executor.accepted(Mock(result=lambda: handle), 1)
    handle.cancel_goal_async.assert_called_once()
    assert executor.busy
    executor.cursor = 5
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), 1)
    assert not executor.busy
    assert executor.phase == 'canceled'


def test_stale_goal_response_is_canceled_without_replacing_current_handle():
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.request_id = 2
    executor.handle = 'current'
    late = Mock(accepted=True)
    executor.accepted(Mock(result=lambda: late), 1)
    late.cancel_goal_async.assert_called_once()
    assert executor.handle == 'current'


def test_early_success_does_not_complete_a_sparse_route():
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.request_id = 1
    executor.handle = Mock()
    executor.stop_state = None
    executor.phase = 'following'
    executor.ready = Mock()
    executor.check_horizon = Mock()
    executor.tracker = SimpleNamespace(length=20.0, progress=3.0)
    executor.cursor = 0
    executor.report = Mock()
    executor.plan_pub = Mock()
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), 1)
    assert executor.phase == 'blocked'


def test_coverage_sensor_subscriptions_keep_only_latest_snapshot():
    import rclpy
    from rclpy.qos import HistoryPolicy, ReliabilityPolicy
    from frontier_explorer.coverage_manager import CoverageManager
    rclpy.init()
    manager = CoverageManager()
    try:
        profiles = {subscription.topic_name: subscription.qos_profile for subscription in manager.subscriptions}
        for topic in ('/scan', '/local_costmap/costmap'):
            assert profiles[topic].depth == 1
            assert profiles[topic].history == HistoryPolicy.KEEP_LAST
            assert profiles[topic].reliability == ReliabilityPolicy.BEST_EFFORT
    finally:
        manager.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('changed', ['data', 'origin', 'clearance'])
def test_costmap_reuses_only_identical_validated_geometry(monkeypatch, changed):
    from copy import deepcopy
    from test_coverage_path import make_grid, FOOTPRINT
    import frontier_explorer.coverage_execution as module
    constructor = Mock(return_value=Mock())
    monkeypatch.setattr(module, 'FreeSpaceValidator', constructor)
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = True
    executor.local = None
    executor.footprint = FOOTPRINT
    executor.clearance = 0.25
    first = make_grid()
    first.header.stamp.sec = 1
    executor.on_costmap(first)
    second = deepcopy(first)
    second.header.stamp.sec = 2
    executor.on_costmap(second)
    assert constructor.call_count == 1
    assert executor.local_stamp.nanoseconds == 2_000_000_000
    third = deepcopy(second)
    if changed == 'data':
        third.data[5000] = 100
    elif changed == 'origin':
        third.info.origin.position.x += 0.05
    else:
        executor.clearance = 0.30
    executor.on_costmap(third)
    assert constructor.call_count == 2


def test_idle_costmap_does_not_compare_or_validate_grid(monkeypatch):
    from test_coverage_path import make_grid, FOOTPRINT
    import frontier_explorer.coverage_execution as module

    class UncomparedData:
        def __eq__(self, other):
            raise AssertionError('Idle callback compared the grid contents')

    constructor = Mock()
    monkeypatch.setattr(module, 'FreeSpaceValidator', constructor)
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = False
    executor.local = Mock()
    executor.footprint = FOOTPRINT
    executor.clearance = 0.25
    grid = make_grid()
    first = SimpleNamespace(header=grid.header, info=grid.info, data=UncomparedData())
    executor.local_grid = first
    executor.local_config = (FOOTPRINT, 0.25)
    latest = SimpleNamespace(header=grid.header, info=grid.info, data=UncomparedData())
    executor.on_costmap(first)
    executor.on_costmap(latest)
    constructor.assert_not_called()
    assert executor.local_grid is first
    assert executor.pending_local_grid is latest
    assert time.monotonic() - executor.local_received < 1.0


@pytest.mark.parametrize('invalid', [False, True])
def test_ready_validates_latest_idle_costmap_before_motion(monkeypatch, invalid):
    from copy import deepcopy
    from rclpy.time import Time
    from test_coverage_path import make_grid, FOOTPRINT
    import frontier_explorer.coverage_execution as module
    validator = Mock()
    constructor = Mock(return_value=validator, side_effect=ValueError('Invalid grid') if invalid else None)
    monkeypatch.setattr(module, 'FreeSpaceValidator', constructor)
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = False
    executor.local = Mock()
    executor.footprint = FOOTPRINT
    executor.clearance = 0.25
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=2_000_000_000)
    executor.scan_received = time.monotonic()
    executor.scan_stamp = Time(nanoseconds=2_000_000_000)
    executor.other_navigation = {}
    first = make_grid()
    first.header.stamp.sec = 1
    latest = deepcopy(first)
    latest.header.stamp.sec = 2
    latest.data[5000] = 100
    executor.on_costmap(first)
    executor.on_costmap(latest)
    constructor.assert_not_called()
    if invalid:
        with pytest.raises(ValueError, match='fresh full local costmap'):
            executor.ready()
        assert executor.local is None
    else:
        executor.ready()
        assert executor.local is validator
        assert executor.local_grid is latest
        executor.ready()
    constructor.assert_called_once_with(latest, FOOTPRINT, 0.25, occupied_threshold=100)
    assert executor.pending_local_grid is None


@pytest.mark.parametrize('stale', ['reception', 'timestamp'])
def test_ready_rejects_stale_idle_costmap_without_validation(monkeypatch, stale):
    from rclpy.time import Time
    from test_coverage_path import make_grid, FOOTPRINT
    import frontier_explorer.coverage_execution as module
    constructor = Mock()
    monkeypatch.setattr(module, 'FreeSpaceValidator', constructor)
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = False
    executor.local = Mock()
    executor.footprint = FOOTPRINT
    executor.clearance = 0.25
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=2_000_000_000)
    executor.scan_received = time.monotonic()
    executor.scan_stamp = Time(nanoseconds=2_000_000_000)
    executor.other_navigation = {}
    grid = make_grid()
    grid.header.stamp.sec = 2 if stale == 'reception' else 0
    executor.on_costmap(grid)
    if stale == 'reception':
        executor.local_received -= 2.0
    with pytest.raises(ValueError, match='costmap'):
        executor.ready()
    constructor.assert_not_called()


def test_dense_coverage_start_does_not_copy_route_on_ros_callback(monkeypatch):
    from concurrent.futures import Future
    from nav_msgs.msg import Path as PathMessage
    from rclpy.time import Time
    from test_coverage_path import make_path
    import frontier_explorer.coverage_execution as module
    original_copy = module.deepcopy

    def bounded_copy(value):
        if isinstance(value, PathMessage) and len(value.poses) > 5000:
            raise AssertionError('Dense route copied inside the ROS callback')
        return original_copy(value)

    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = False
    executor.report = Mock()
    executor.ready = Mock()
    executor.validate = Mock()
    route = make_path([(1, 1, 0), (2, 1, 0)])
    route.poses *= 3000
    executor.robot_pose = Mock(return_value=route.poses[0])
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=1_000_000_000)
    executor.navigator = Mock()
    executor.follower = Mock()
    executor.pause_explore = Mock()
    executor.send = Mock()
    executor.join_pool = Mock()
    executor.join_pool.submit.return_value = Future()
    monkeypatch.setattr(module, 'deepcopy', bounded_copy)
    executor.start(route)
    assert executor.route is route
    assert executor.busy
    assert executor.phase == 'validating_start'
    executor.validate.assert_not_called()
    executor.send.assert_not_called()
    executor.join_pool.submit.assert_called_once_with(executor.validate_initial_route, route, False)
    executor.join_future.set_result(route)
    executor.guard()
    executor.send.assert_called_once()
    client, goal = executor.send.call_args.args
    assert client is executor.navigator
    assert goal.pose.pose == route.poses[0].pose
    assert goal.pose.header.frame_id == route.header.frame_id
    assert executor.phase == 'navigating_ingress'


@pytest.mark.parametrize('scan_nanoseconds,expected_age', [(0, '2.000'), (3_000_000_000, '-1.000')])
def test_scan_timestamp_guard_preserves_limits_and_reports_age(scan_nanoseconds, expected_age):
    from rclpy.time import Time
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.local = Mock()
    executor.local_received = executor.scan_received = time.monotonic()
    executor.scan_stamp = Time(nanoseconds=scan_nanoseconds)
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value.nanoseconds = 2_000_000_000
    with pytest.raises(ValueError, match=f'ROS age={expected_age}'):
        executor.ready()


def test_ingress_waits_for_stop_then_submits_only_the_coverage_route():
    from copy import deepcopy
    from rclpy.time import Time
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=1_000_000_000)
    executor.request_id = 1
    executor.handle = Mock()
    executor.stop_state = None
    executor.phase = 'navigating_ingress'
    executor.min_radius = 0.20
    executor.route = make_path([(2, 1, 0), (2.5, 1, 0), (3, 1, 0)])
    route_before = deepcopy(executor.route)
    executor.validate = Mock()
    executor.ready = Mock()
    executor.check_horizon = Mock()
    executor.plan_pub = Mock()
    executor.report = Mock()
    executor.send = Mock()
    executor.follower = Mock()
    executor.robot_pose = Mock(return_value=executor.route.poses[0])
    executor.stop_confirmed = Mock(return_value=False)
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), 1)
    assert executor.phase == 'waiting_ingress_stop'
    executor.check_ingress_stop()
    executor.send.assert_not_called()
    executor.stop_confirmed.return_value = True
    executor.check_ingress_stop()
    executor.send.assert_called_once()
    sent_goal = executor.send.call_args.args[1]
    assert sent_goal.controller_id == 'CoverageFollowPath'
    assert sent_goal.goal_checker_id == 'coverage_goal_checker'
    assert [stamped.pose for stamped in sent_goal.path.poses] == [
        stamped.pose for stamped in route_before.poses]
    assert sent_goal.path.header.stamp == Time(nanoseconds=1_000_000_000).to_msg()
    assert all(stamped.header.stamp == sent_goal.path.header.stamp
               for stamped in sent_goal.path.poses)
    assert executor.route == route_before
    executor.validate.assert_called_once_with(route_before, ingress=True)


@pytest.mark.parametrize('deviation', [0.30, 0.60])
def test_coverage_tracking_deviation_warns_without_fabricating_a_return_path(deviation):
    from geometry_msgs.msg import Transform
    from rclpy.time import Time
    from frontier_explorer.navigation_metrics import OrderedPathTracker
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.execution_path = make_path([(1, 2, 0), (2, 2, 0), (3, 2, 0)])
    executor.coverage_start_index = 0
    executor.robot_pose = Mock(return_value=make_path([(1.2, 2.0 + deviation, 0)]).poses[0])
    executor.tracker = OrderedPathTracker([(1, 2), (2, 2), (3, 2)], search_distance=0.6)
    executor.cursor = 0
    executor.node = Mock()
    executor.report = Mock()
    executor.last_feedback = 0.0
    executor.phase = 'following'
    executor.local = Mock(frame='map')
    executor.local_stamp = Time()
    executor.tf = Mock()
    transform = Transform()
    transform.rotation.w = 1.0
    executor.tf.lookup_transform.return_value.transform = transform
    executor.check_horizon()
    executor.node.get_logger.return_value.warn.assert_called_once()
    checked_path = executor.local.check_path.call_args.args[0]
    assert len(checked_path.poses) == 1
    assert checked_path.poses[0].pose.position.y == 2.0 + deviation
    assert executor.report.call_args.kwargs['tracking_warning']
    assert executor.report.call_args.kwargs['stage'] == 'coverage'


@pytest.mark.parametrize('external_action', ['navigate_to_pose', 'navigate_through_poses'])
def test_coverage_owns_only_its_ordinary_ingress_goal(external_action):
    from action_msgs.msg import GoalStatusArray
    from nav2_msgs.action import NavigateToPose
    from unique_identifier_msgs.msg import UUID
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.navigator = Mock()
    executor.request_id = 0
    executor.busy = True
    executor.other_navigation = {}
    executor.cancel = Mock()
    executor.send(executor.navigator, NavigateToPose.Goal())
    own_uuid = executor.navigator.send_goal_async.call_args.kwargs['goal_uuid']
    status = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    status.goal_info.goal_id = own_uuid
    executor.on_navigation('navigate_to_pose', GoalStatusArray(status_list=[status]))
    assert not executor.other_navigation['navigate_to_pose']
    executor.cancel.assert_not_called()
    status.goal_info.goal_id = UUID(uuid=[42] * 16)
    executor.on_navigation(external_action, GoalStatusArray(status_list=[status]))
    assert executor.other_navigation[external_action]
    executor.cancel.assert_called_once()


def test_operator_cancel_after_ordinary_approach_never_starts_coverage():
    executor = segmented_executor()
    executor.phase = 'navigating_ingress'
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), 1)
    assert executor.phase == 'waiting_ingress_stop'
    executor.cancel()
    executor.guard()
    assert executor.phase == 'canceled'
    assert not executor.busy
    assert executor.send.call_count == 1


def recovery_executor():
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.ready = Mock()
    executor.local = Mock(frame='map')
    executor.robot_pose = Mock(return_value=make_path([(2, 2, 0)]).poses[0])
    executor.execution_path = make_path([(1, 2, 0), (2, 2, 0), (3, 2, 0)])
    executor.cursor = 1
    executor.recovery_attempts = 0
    executor.recovery_max_attempts = 2
    executor.recovery_backup_enabled = True
    executor.recovery_backup_distance = 0.15
    executor.recovery_backup_speed = 0.05
    executor.backup = Mock()
    executor.backup.server_is_ready.return_value = True
    executor.report = Mock()
    executor.send = Mock()
    executor.finish = Mock()
    return executor


def test_recovery_backup_checks_swept_space_and_retains_progress():
    executor = recovery_executor()
    executor.begin_recovery()
    assert executor.phase == 'backing_up'
    assert executor.recovery_attempts == 1
    checked = executor.local.check_path.call_args.args[0]
    assert checked.poses[0].pose.position.x == 2.0
    assert checked.poses[-1].pose.position.x == pytest.approx(1.85)
    assert executor.recovery_route.poses[0].pose.position.x == 2.0
    client, goal = executor.send.call_args.args
    assert client is executor.backup
    assert goal.target.x == 0.15
    assert goal.speed == 0.05
    assert goal.time_allowance.sec == 8
    executor.finish.assert_not_called()


def segmented_executor():
    from rclpy.time import Time
    from test_coverage_path import make_path
    executor = recovery_executor()
    executor.validate = Mock()
    del executor.finish
    executor.route = make_path([(2, 2, 0), (2.5, 2, 0),
                                (3, 2.5, math.pi), (2.5, 2.5, math.pi),
                                (2, 3, 0), (3, 3, 0)])
    executor.work_sections = [[0, 1], [2, 3], [4, 5]]
    executor.sections = [(0, 1, 1), (2, 3, 1), (4, 5, 1)]
    executor.section_index = 0
    executor.cursor = 0
    executor.busy = True
    executor.handle = None
    executor.stop_state = None
    executor.cancel_sent = False
    executor.request_id = 1
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=100)
    executor.navigator = Mock()
    executor.last_ros_time = 99
    executor.last_feedback = 0.0
    executor.plan_pub = Mock()
    executor.control_plan_pub = Mock()
    executor.follower = Mock()
    executor.motion_stamp = Time(nanoseconds=100)
    executor.motion_received = time.monotonic()
    executor.motion_linear = 0.20
    executor.motion_angular = 0.0
    executor.last_motion_direction = None
    executor.begin_following(executor.route)
    return executor


def complete_motion_section(executor, remaining_distance=0.0):
    from copy import deepcopy
    endpoint = executor.sections[executor.section_index][1]
    executor.robot_pose.return_value = deepcopy(executor.execution_path.poses[endpoint])
    from frontier_explorer.coverage_path import pose_xy_yaw
    yaw = pose_xy_yaw(executor.robot_pose.return_value.pose)[2]
    executor.robot_pose.return_value.pose.position.x -= math.cos(yaw) * remaining_distance
    executor.robot_pose.return_value.pose.position.y -= math.sin(yaw) * remaining_distance
    executor.section_tracker.progress = executor.section_tracker.length - remaining_distance
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), 1)


@pytest.mark.parametrize('remaining_distance', [0.0, 0.04])
def test_work_sections_wait_for_actual_stop_before_ordinary_transit(remaining_distance):
    executor = segmented_executor()
    assert executor.send.call_args.args[1].controller_id == 'CoverageFollowPath'
    assert len(executor.send.call_args.args[1].path.poses) == 2
    complete_motion_section(executor, remaining_distance)
    assert executor.phase == 'waiting_work_stop'
    assert executor.cursor == 2
    executor.guard()
    assert executor.send.call_count == 1
    executor.motion_linear = 0.0
    executor.stopped_since = time.monotonic() - 0.4
    executor.guard()
    assert executor.phase == 'navigating_ingress'
    assert executor.send.call_args.args[0] is executor.navigator
    assert executor.send.call_args.args[1].pose.pose == executor.route.poses[2].pose
    executor.robot_pose.return_value = executor.route.poses[2]
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), 1)
    assert executor.phase == 'waiting_ingress_stop'
    executor.stopped_since = time.monotonic() - 0.4
    executor.guard()
    assert executor.send.call_args.args[1].controller_id == 'CoverageFollowPath'
    assert executor.send.call_args.args[1].goal_checker_id == 'coverage_goal_checker'
    assert [stamped.pose for stamped in executor.plan_pub.publish.call_args.args[0].poses] == [
        stamped.pose for stamped in executor.route.poses[2:4]]
    assert executor.section_offset == pytest.approx(0.5)
    complete_motion_section(executor, remaining_distance)
    assert executor.phase == 'waiting_work_stop'
    assert executor.cursor == 4
    executor.stopped_since = time.monotonic() - 0.4
    executor.guard()
    assert executor.send.call_count == 4
    assert executor.send.call_args.args[0] is executor.navigator
    assert executor.send.call_args.args[1].pose.pose == executor.route.poses[4].pose


def test_operator_cancel_at_work_stop_never_starts_next_section():
    executor = segmented_executor()
    complete_motion_section(executor)
    executor.cancel()
    executor.guard()
    assert executor.phase == 'canceled'
    assert not executor.busy
    assert executor.remaining_path().poses == executor.route.poses[2:]
    assert executor.remaining_work_sections() == [[0, 1], [2, 3]]
    assert executor.send.call_count == 1


def test_work_sections_never_send_transit_geometry_to_coverage():
    executor = segmented_executor()
    assert [stamped.pose for stamped in executor.send.call_args.args[1].path.poses] == [
        stamped.pose for stamped in executor.route.poses[:2]]
    assert executor.tracker.length == pytest.approx(0.5)
    assert executor.total_work_length == pytest.approx(2.0)
    assert executor.report.call_args_list[-2].kwargs['distance_remaining'] == pytest.approx(2.0)
    assert executor.remaining_work_sections() == [[0, 1], [2, 3], [4, 5]]


@pytest.mark.parametrize('section_index', [0, 1])
def test_work_sections_refresh_all_stamps_without_mutating_route(section_index):
    from copy import deepcopy
    from rclpy.time import Time
    executor = segmented_executor()
    route_before = deepcopy(executor.route)
    first_path = executor.send.call_args.args[1].path
    first_path_before = deepcopy(first_path)
    executor.section_index = section_index
    start, end, _ = executor.sections[section_index]
    executor.robot_pose.return_value = executor.route.poses[start]
    now = Time(nanoseconds=2_000_000_000)
    executor.node.get_clock.return_value.now.return_value = now
    executor.send_motion_section()
    sent_path = executor.send.call_args.args[1].path
    assert sent_path.header.stamp == now.to_msg()
    assert all(stamped.header.stamp == now.to_msg() for stamped in sent_path.poses)
    assert sent_path.header.frame_id == executor.route.header.frame_id
    assert [stamped.pose for stamped in sent_path.poses] == [
        stamped.pose for stamped in executor.route.poses[start:end + 1]]
    assert all(stamped is not original for stamped, original in zip(
        sent_path.poses, executor.route.poses[start:end + 1]))
    assert executor.plan_pub.publish.call_args.args[0] == sent_path
    assert executor.control_plan_pub.publish.call_args.args[0] == sent_path
    assert executor.route == route_before
    assert first_path == first_path_before


def test_work_stop_does_not_confirm_stop_from_one_odom_sample():
    executor = segmented_executor()
    complete_motion_section(executor)
    executor.motion_linear = 0.0
    executor.motion_received = time.monotonic() - 0.4
    executor.stopped_since = executor.motion_received
    executor.guard()
    assert executor.phase == 'waiting_work_stop'
    assert executor.send.call_count == 1
    executor.motion_received = time.monotonic()
    executor.guard()
    assert executor.send.call_args.args[0] is executor.navigator


@pytest.mark.parametrize('reason', ['stale', 'future', 'timeout'])
def test_work_stop_refuses_unconfirmed_stop(reason):
    from rclpy.time import Time
    executor = segmented_executor()
    complete_motion_section(executor)
    if reason == 'stale':
        executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=3000000100)
    elif reason == 'future':
        executor.motion_stamp = Time(nanoseconds=2000000100)
    else:
        executor.started = time.monotonic() - 6.0
    executor.guard()
    executor.guard()
    assert executor.phase == 'blocked'
    assert not executor.busy
    assert executor.send.call_count == 1


@pytest.mark.parametrize('reason', ['sensor', 'obstacle', 'budget'])
def test_recovery_refuses_stale_blocked_or_exhausted_backup(reason):
    executor = recovery_executor()
    if reason == 'sensor':
        executor.ready.side_effect = ValueError('scan is stale')
    elif reason == 'obstacle':
        executor.local.check_path.side_effect = ValueError('backup space is occupied')
    else:
        executor.recovery_attempts = 2
    executor.begin_recovery()
    executor.send.assert_not_called()
    assert executor.finish.call_args.args[0] == 'blocked'


def test_follow_path_abort_recovers_only_after_confirmed_action_end():
    executor = recovery_executor()
    executor.request_id = 1
    executor.handle = Mock()
    executor.stop_state = None
    executor.phase = 'following'
    executor.recovery_enabled = True
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_ABORTED)), 1)
    assert executor.handle is None
    assert executor.phase == 'backing_up'
    executor.send.assert_called_once()


def test_follow_path_abort_uses_ordinary_approach_without_automatic_backup():
    from concurrent.futures import Future
    from nav2_msgs.action import FollowPath
    from rclpy.time import Time
    executor = recovery_executor()
    executor.busy = True
    executor.request_id = 1
    executor.handle = Mock()
    executor.stop_state = None
    executor.phase = 'following'
    executor.recovery_enabled = True
    executor.recovery_backup_enabled = False
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=100)
    executor.last_ros_time = 99
    executor.navigator = Mock()
    executor.follower = Mock()
    executor.join_future = None
    executor.join_pool = Mock()
    validation = Future()
    executor.join_pool.submit.return_value = validation
    executor.result(Mock(result=lambda: SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED, result=FollowPath.Result())), 1)
    assert executor.handle is None
    assert executor.phase == 'validating_recovery'
    assert executor.cursor == 1
    assert executor.recovery_attempts == 1
    assert executor.recovery_route.poses == executor.execution_path.poses[1:]
    assert len(executor.local.check_path.call_args.args[0].poses) == 1
    executor.backup.server_is_ready.assert_not_called()
    executor.send.assert_not_called()
    executor.join_pool.submit.assert_called_once_with(executor.validate_recovery_route)
    retained = executor.recovery_route
    validation.set_result(retained)
    executor.guard()
    assert executor.phase == 'navigating_ingress'
    assert executor.remaining_path() is retained
    client, goal = executor.send.call_args.args
    assert client is executor.navigator
    assert goal.pose.pose == retained.poses[0].pose
    executor.finish.assert_not_called()


def test_operator_cancel_never_triggers_backup_recovery():
    executor = recovery_executor()
    executor.request_id = 1
    executor.handle = Mock()
    executor.stop_state = ('canceled', 'operator stop')
    executor.phase = 'following'
    executor.recovery_enabled = True
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), 1)
    executor.send.assert_not_called()
    executor.finish.assert_called_once_with('canceled', 'operator stop')


def test_blocked_horizon_cancels_and_retains_resume_cursor():
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy, executor.stop_state, executor.phase = True, None, 'following'
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value.nanoseconds = 100
    executor.last_ros_time = 99
    executor.ready = Mock()
    executor.check_horizon = Mock(side_effect=ValueError('obstacle'))
    executor.report, executor.handle = Mock(), Mock()
    executor.cancel_sent = False
    executor.cursor = 1
    executor.execution_path = make_path([(1, 1, 0), (2, 1, 0), (3, 1, 0)])
    executor.guard()
    executor.handle.cancel_goal_async.assert_called_once()
    assert executor.busy
    assert executor.stop_state == ('blocked', 'obstacle')
    assert executor.remaining_path().poses == executor.execution_path.poses[1:]


@pytest.mark.parametrize('phase', ['validating_ingress', 'validating_recovery'])
@pytest.mark.parametrize('worker_consumed', [False, True])
def test_canceled_large_ingress_validation_cannot_send_follow_path(phase, worker_consumed):
    from concurrent.futures import Future
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = True
    executor.phase = phase
    executor.stop_state = ('canceled', 'operator stop')
    executor.handle = None
    executor.join_future = None if worker_consumed else Future()
    executor.plan_pub = Mock()
    executor.report = Mock()
    executor.cursor = 0
    executor.send = Mock()
    executor.guard()
    if not worker_consumed:
        assert executor.busy
        executor.join_future.set_result('late validated route')
        executor.guard()
    assert not executor.busy
    assert executor.phase == 'canceled'
    executor.send.assert_not_called()


@pytest.mark.parametrize(('phase', 'failure'), [
    ('validating_ingress', 'validation'),
    ('validating_recovery', 'validation'),
    ('validating_ingress', 'readiness'),
    ('validating_recovery', 'readiness'),
    ('validating_ingress', 'horizon'),
])
def test_failed_validation_handoff_blocks_without_crashing_or_sending_motion(phase, failure):
    from concurrent.futures import Future
    from rclpy.time import Time
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = True
    executor.phase = phase
    executor.stop_state = None
    executor.handle = None
    executor.cancel_sent = False
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=100)
    executor.last_ros_time = 99
    executor.started = time.monotonic()
    executor.cursor = 1
    executor.route = make_path([(1, 1, 0), (2, 1, 0), (3, 1, 0)])
    executor.execution_path = executor.route
    executor.plan_pub = Mock()
    executor.report = Mock()
    executor.send = Mock()
    executor.ready = Mock()
    executor.check_horizon = Mock()
    executor.join_future = Future()
    if failure == 'validation':
        executor.join_future.set_exception(ValueError('validation failed'))
    else:
        executor.join_future.set_result(executor.route)
        if failure == 'readiness':
            executor.ready.side_effect = [None, ValueError('readiness failed')]
        else:
            executor.check_horizon.side_effect = ValueError('horizon failed')
    executor.guard()
    executor.guard()
    assert not executor.busy
    assert executor.phase == 'blocked'
    assert executor.join_future is None
    assert executor.remaining_path().poses == executor.route.poses[1:]
    executor.report.assert_called_with('blocked', f'{failure} failed', path_index=1)
    executor.send.assert_not_called()


def test_new_obstacle_during_backup_cancels_without_replanning():
    executor = recovery_executor()
    executor.busy = True
    executor.stop_state = None
    executor.phase = 'backing_up'
    executor.node = Mock()
    executor.node.get_clock.return_value.now.return_value.nanoseconds = 100
    executor.last_ros_time = 99
    executor.started = time.monotonic()
    executor.backup_start = executor.robot_pose('map')
    executor.handle = Mock()
    executor.cancel_sent = False
    executor.local.check_path.side_effect = ValueError('new rear obstacle')
    executor.guard()
    executor.handle.cancel_goal_async.assert_called_once()
    assert executor.stop_state == ('blocked', 'new rear obstacle')
    executor.send.assert_not_called()


def test_execute_revalidation_preserves_partial_coverage_metadata(monkeypatch):
    import rclpy
    from concurrent.futures import Future
    from frontier_explorer.coverage_manager import CoverageManager
    from test_coverage_path import make_path
    rclpy.init()
    manager = CoverageManager()
    try:
        route = make_path([(1, 1, 0), (2, 1, 0)])
        route.poses *= 3000
        manager._cached_preview_details = dict(partial_coverage=True, unconnected_segment_count=7,
                                               omitted_work_length_m=12.5, swath_count=18)
        submit = Mock(return_value=Future())
        monkeypatch.setattr(manager._layout_pool, 'submit', submit)
        manager._start_checked_execution(route)
        details = submit.call_args.args[2]
        assert details['partial_coverage']
        assert details['unconnected_segment_count'] == 7
        assert details['omitted_work_length_m'] == 12.5
        assert details is not manager._cached_preview_details
        assert manager._pending_execution is False
        assert not hasattr(manager._execution, 'planner')
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_blocked_valid_preview_remains_manually_retryable_without_automatic_motion():
    import json
    import rclpy
    from frontier_explorer.coverage_manager import CoverageManager
    from test_coverage_path import make_path
    rclpy.init()
    manager = CoverageManager()
    try:
        manager._cached_path = make_path([(1, 1, 0), (2, 1, 0)])
        manager._last_preview_valid = True
        manager._status_pub.publish = Mock()
        manager._execution.start = Mock()
        manager._publish_status('blocked', 'Laser scan is stale')
        payload = json.loads(manager._status_pub.publish.call_args.args[0].data)
        assert payload['can_execute']
        manager._execution.start.assert_not_called()
        manager._last_preview_valid = False
        manager._publish_status('stale_preview', 'Preview changed')
        payload = json.loads(manager._status_pub.publish.call_args.args[0].data)
        assert not payload['can_execute']
        manager._execution.start.assert_not_called()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_zone_change_discards_interrupted_route():
    import rclpy
    from frontier_explorer.coverage_manager import CoverageManager
    from test_coverage_path import make_path
    rclpy.init()
    manager = CoverageManager()
    try:
        manager._execution.route = make_path([(1, 1, 0), (2, 1, 0)])
        manager._execution.phase = 'canceled'
        manager._on_set_zone('[{"x":0,"y":0},{"x":5,"y":0},{"x":5,"y":5}]')
        assert manager._execution.remaining_path() is None
        assert not manager._last_preview_valid
    finally:
        manager.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('use_sim_time', [True, False])
def test_directional_preview_uses_recorded_map_and_zone(use_sim_time):
    import numpy as np
    import rclpy
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Point32
    from frontier_explorer.coverage_manager import CoverageManager
    from test_coverage_path import make_grid, make_path
    rclpy.init()
    manager = CoverageManager()
    try:
        manager.set_parameters([Parameter('use_sim_time', value=use_sim_time),
                                Parameter('layout_mode', value='directional')])
        with np.load(Path(__file__).parent / 'data/ingress_replay.npz', allow_pickle=False) as saved:
            grid = make_grid()
            grid.info.height, grid.info.width = saved['occupancy'].shape
            grid.info.resolution = float(saved['resolution'])
            grid.data = saved['occupancy'].ravel().tolist()
            grid.info.origin = make_path([saved['origin']]).poses[0].pose
        manager._last_map_msg = grid
        manager._last_map_received = time.monotonic()
        manager._polygon_msg.header = grid.header
        zone = [(-7.273040295, -4.674580574), (-7.273040295, 1.125419497),
                (-3.623040438, 1.125419497), (-3.623040438, -4.674580574),
                (-6.123040676, -4.674580574), (-6.273040295, -4.524580479),
                (-6.423040390, -4.674580574), (-7.273040295, -4.674580574)]
        manager._polygon_msg.polygon.points = [Point32(x=position_x, y=position_y) for position_x, position_y in zone]
        manager._start_preview()
        assert manager._preview_pending
        deadline = time.monotonic() + 30.0
        while manager._preview_pending and time.monotonic() < deadline:
            manager._last_map_received = time.monotonic()
            rclpy.spin_once(manager, timeout_sec=0.02)
        assert manager._state == 'preview_ready'
        assert manager._last_preview_valid
        assert len(manager._cached_path.poses) > 800
        assert manager._cached_preview_details['work_sections']
        assert manager._cached_preview_details['transit_mode'] == 'navigate_to_pose'
        from frontier_explorer.coverage_path import work_paths
        display_work = work_paths(manager._display_path, manager._display_sections)
        assert len(display_work) == len(manager._cached_preview_details['work_sections'])
        assert all(len(work.poses) >= 2 for work in display_work)
        manager._execution.start = Mock()
        manager._start_execution()
        assert (manager._execution.start.call_args.kwargs['work_sections']
            == manager._cached_preview_details['work_sections'])
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_invalid_zone_extraction_cannot_reuse_previous_polygon():
    import rclpy
    from geometry_msgs.msg import Point32
    from frontier_explorer.coverage_manager import CoverageManager
    from test_coverage_path import make_grid
    rclpy.init()
    manager = CoverageManager()
    try:
        manager._last_map_msg = make_grid()
        manager._polygon_msg.polygon.points = [Point32(x=1.0, y=1.0)] * 5
        manager._map_processor.extract_boundary_and_obstacles = Mock(return_value=(None, []))
        manager._on_set_zone('[{"x":20,"y":20},{"x":25,"y":20},{"x":25,"y":25}]')
        assert not manager._polygon_msg.polygon.points
        assert not manager._last_preview_valid
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_collapsed_headland_is_rejected_before_submitting_preview():
    import rclpy
    from geometry_msgs.msg import Point32
    from frontier_explorer.coverage_manager import CoverageManager
    from test_coverage_path import make_path
    rclpy.init()
    manager = CoverageManager()
    try:
        manager._polygon_msg.polygon.points = [Point32(x=position_x, y=position_y)
            for position_x, position_y in [(0.0, 0.0), (2.7, 0.0), (2.7, 4.45), (0.0, 4.45), (0.0, 0.0)]]
        manager._compute_client = Mock()
        manager._publish_status = Mock()
        manager._last_preview_valid = True
        manager._cached_path = make_path([(1, 1, 0), (2, 1, 0)])
        manager._start_preview()
        manager._compute_client.send_goal_async.assert_not_called()
        assert not manager._busy()
        assert not manager._last_preview_valid
        assert not manager._cached_path.poses
        state, message = manager._publish_status.call_args.args
        assert state == 'polygon_invalid'
        assert '2.70 x 4.45 m' in message
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_canceled_pending_preview_cannot_enable_execution():
    import rclpy
    from frontier_explorer.coverage_manager import CoverageManager
    rclpy.init()
    manager = CoverageManager()
    try:
        manager._preview_pending = True
        manager._cancel_active_goals()
        handle = Mock(accepted=True)
        manager._on_preview_goal_response(Mock(result=lambda: handle))
        handle.cancel_goal_async.assert_called_once()
        assert manager._busy()
        manager._on_preview_result(Mock())
        assert not manager._busy()
        assert not manager._last_preview_valid
        assert manager._state == 'canceled'
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_canceled_directional_worker_cannot_publish_a_stale_preview():
    import rclpy
    from concurrent.futures import Future
    from frontier_explorer.coverage_manager import CoverageManager
    from test_coverage_path import make_path
    rclpy.init()
    manager = CoverageManager()
    try:
        future = Future()
        manager._layout_future = future
        manager._preview_pending = True
        manager._cancel_active_goals()
        assert manager._busy()
        assert manager._layout_cancel.is_set()
        future.set_result((make_path([(1, 1, 0), (2, 1, 0)]), {}))
        manager._poll_directional_preview()
        assert manager._state == 'canceled'
        assert not manager._last_preview_valid
        assert not manager._cached_path.poses
        assert not manager._busy()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('use_sim_time', ['true', 'false'])
@pytest.mark.parametrize('namespace', ['', 'robot'])
def test_coverage_profile_is_shared_without_changing_clock(use_sim_time, namespace):
    import json
    root = Path(__file__).resolve().parents[1]
    spec = spec_from_file_location('navigation_launch', root / 'launch/navigation_launch.py')
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(use_sim_time=use_sim_time, namespace=namespace)
    params = module.CoverageTrialParameters(
        overlay=str(root / 'config/coverage_sim.yaml'),
        namespace=LaunchConfiguration('namespace'),
        source_file=str(root / 'config/explore.yaml'), root_key=LaunchConfiguration('namespace'),
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')}, convert_types=True)
    with open(params.perform(context)) as stream:
        loaded = yaml.safe_load(stream)
    if namespace:
        loaded = loaded[namespace]
    controller = loaded['controller_server']['ros__parameters']
    assert controller['use_sim_time'] == (use_sim_time == 'true')
    assert 'CoverageFollowPath' in controller['controller_plugins']
    assert loaded['local_costmap']['local_costmap']['ros__parameters']['footprint'].startswith('[[')
    assert controller['CoverageFollowPath']['motion_model'] == 'Ackermann'
    assert controller['CoverageFollowPath']['vx_min'] == 0.0
    assert controller['CoverageFollowPath']['vx_max'] == 0.20
    assert controller['controller_plugins'] == ['FollowPath', 'CoverageFollowPath']
    assert controller['goal_checker_plugins'] == ['goal_checker', 'coverage_goal_checker']
    assert not loaded['coverage_manager']['ros__parameters']['recovery_backup_enabled']
    assert loaded['planner_server']['ros__parameters']['planner_plugins'] == ['GridBased']
    planner = loaded['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner/SmacPlannerLattice'
    assert not planner['allow_reverse_expansion']
    assert not planner['smooth_path']
    assert not planner['cache_obstacle_heuristic']
    assert planner['max_planning_time'] == 2.0
    assert {'downsample_costmap', 'downsampling_factor', 'cost_travel_multiplier',
            'use_final_approach_orientation', 'smoother'}.isdisjoint(planner)
    with Path(planner['lattice_filepath']).open() as stream:
        lattice = json.load(stream)['lattice_metadata']
    assert lattice['motion_model'] == 'diff'
    assert lattice['turning_radius'] == 0.5
    for costmap in ('global_costmap', 'local_costmap'):
        assert lattice['grid_resolution'] == loaded[costmap][costmap]['ros__parameters']['resolution']
    work = controller['CoverageFollowPath']
    assert work['max_robot_pose_search_dist'] == 0.6
    assert work['prune_distance'] == 1.2
    assert work['GoalCritic']['threshold_to_consider'] == 0.10
    assert work['PathFollowCritic']['threshold_to_consider'] == 0.10
    assert work['VelocityDeadbandCritic']['deadband_velocities'] == [0.18, 0.0, 0.0]
    assert work['VelocityDeadbandCritic']['cost_weight'] == 35.0
    for controller_id in controller['controller_plugins']:
        profile = controller[controller_id]
        assert profile['plugin'] == 'nav2_mppi_controller::MPPIController'
        assert {'validate_control_sequence', 'wheel_radius', 'wheel_separation',
                'wheel_minimum_velocity'}.isdisjoint(profile)
        assert 'consider_filled_footprint' not in profile['CostCritic']
    assert work['AckermannConstraints']['min_turning_r'] == 0.20
    assert loaded['coverage_manager']['ros__parameters']['path_type'] == 'DUBIN'
    assert loaded['coverage_manager']['ros__parameters']['clearance'] == 0.25
    assert controller['FollowPath']['motion_model'] == 'DiffDrive'
    assert controller['FollowPath']['vx_min'] == 0.0
    assert controller['FollowPath']['vx_max'] == 0.40
    assert controller['FollowPath']['max_robot_pose_search_dist'] == 2.5
    assert controller['FollowPath']['prune_distance'] == 1.6
    assert controller['FollowPath']['PathFollowCritic']['threshold_to_consider'] == 0.5
    assert 'VelocityDeadbandCritic' not in controller['FollowPath']['critics']
    assert (loaded['velocity_smoother']['ros__parameters']['max_velocity'][0]
            >= controller['FollowPath']['vx_max'])
    assert loaded['velocity_smoother']['ros__parameters']['min_velocity'][0] == -0.20
    assert controller['FollowPath']['model_dt'] == 1.0 / controller['controller_frequency']
    assert controller['FollowPath']['GoalAngleCritic']['enabled']
    assert (controller['FollowPath']['GoalAngleCritic']['threshold_to_consider']
            <= controller['goal_checker']['xy_goal_tolerance'])
    assert (controller['FollowPath']['PathAngleCritic']['threshold_to_consider']
            == controller['FollowPath']['GoalAngleCritic']['threshold_to_consider'])
    assert 'VelocityDeadbandCritic' not in controller['FollowPath']['critics']


def test_ordered_goal_checker_retains_progress_during_tracking_warning(tmp_path):
        implementation = Path(__file__).resolve().parents[2] / 'coverage_geometry/src/ordered_goal_checker.cpp'
        (tmp_path / 'CMakeLists.txt').write_text('''
cmake_minimum_required(VERSION 3.16)
project(ordered_warning_progress LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
add_executable(ordered_probe ordered_probe.cpp)
target_compile_features(ordered_probe PRIVATE cxx_std_17)
ament_target_dependencies(ordered_probe nav2_controller nav2_costmap_2d pluginlib)
''')
        (tmp_path / 'ordered_probe.cpp').write_text(f'#include "{implementation}"\n' + '''
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <stdexcept>
#include <thread>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("ordered_probe");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->configure();
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ordered_checker_probe");
    relobot::OrderedGoalChecker checker;
    checker.initialize(node, "goal", costmap);
    auto publisher = node->create_publisher<nav_msgs::msg::Path>("/coverage/execution_path", rclcpp::QoS(1).transient_local());
    publisher->on_activate();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node->get_node_base_interface());
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int index = 0; index <= 40; ++index) {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.pose.position.x = 2.0 + 0.025 * index;
        stamped.pose.position.y = 2.0;
        stamped.pose.orientation.w = 1.0;
        path.poses.push_back(stamped);
    }
    publisher->publish(path);
    const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < receive_deadline) {
        rclcpp::spin_some(node->get_node_base_interface());
    }
    auto goal = path.poses.back().pose;
    geometry_msgs::msg::Twist stopped;
    for (int index = 0; index < 40; ++index) {
        auto robot = path.poses[index].pose;
        robot.position.y += 0.30;
        if (checker.isGoalReached(robot, goal, stopped)) {
            throw std::runtime_error("Tracking warning caused premature completion");
        }
    }
    if (!checker.isGoalReached(goal, goal, stopped)) {
        throw std::runtime_error("Ordered progress was lost during nonfatal deviation");
    }
    path.poses.push_back(path.poses.front());
    publisher->publish(path);
    const auto closed_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < closed_deadline) {
        rclcpp::spin_some(node->get_node_base_interface());
    }
    checker.reset();
    if (checker.isGoalReached(path.poses.front().pose, path.poses.back().pose, stopped)) {
        throw std::runtime_error("Closed route completed at its shared start/end");
    }
    node->declare_parameter<std::string>("reverse_goal.path_topic", "/coverage/control_path");
    node->declare_parameter<double>("reverse_goal.xy_goal_tolerance", 0.15);
    node->declare_parameter<double>("reverse_goal.yaw_goal_tolerance", 0.20);
    relobot::OrderedGoalChecker reverse_checker;
    reverse_checker.initialize(node, "reverse_goal", costmap);
    auto control_publisher = node->create_publisher<nav_msgs::msg::Path>(
        "/coverage/control_path", rclcpp::QoS(1).transient_local());
    control_publisher->on_activate();
    path.poses.clear();
    for (int index = 0; index <= 20; ++index) {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.pose.position.x = 2.0 - 0.025 * index;
        stamped.pose.position.y = 2.0;
        stamped.pose.orientation.w = 1.0;
        path.poses.push_back(stamped);
    }
    control_publisher->publish(path);
    const auto reverse_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < reverse_deadline) {
        rclcpp::spin_some(node->get_node_base_interface());
    }
    for (size_t index = 0; index < 14; ++index) {
        if (reverse_checker.isGoalReached(path.poses[index].pose, path.poses.back().pose, stopped)) {
            throw std::runtime_error("Reverse section completed outside its position tolerance");
        }
    }
    auto near_goal = path.poses.back().pose;
    near_goal.position.x += 0.16;
    if (reverse_checker.isGoalReached(near_goal, path.poses.back().pose, stopped)) {
        throw std::runtime_error("Reverse section accepted a 16 cm endpoint error");
    }
    near_goal.position.x = path.poses.back().pose.position.x + 0.14;
    near_goal.orientation.z = std::sin(0.21 * 0.5);
    near_goal.orientation.w = std::cos(0.21 * 0.5);
    if (reverse_checker.isGoalReached(near_goal, path.poses.back().pose, stopped)) {
        throw std::runtime_error("Reverse section accepted a yaw error beyond 0.20 radians");
    }
    near_goal.orientation.z = std::sin(0.19 * 0.5);
    near_goal.orientation.w = std::cos(0.19 * 0.5);
    if (!reverse_checker.isGoalReached(near_goal, path.poses.back().pose, stopped)) {
        throw std::runtime_error("Ordered progress vetoed the configured 15 cm endpoint tolerance");
    }
    for (size_t index = 0; index + 1 < path.poses.size(); ++index) {
        reverse_checker.isGoalReached(path.poses[index].pose, path.poses.back().pose, stopped);
    }
    if (!reverse_checker.isGoalReached(path.poses.back().pose, path.poses.back().pose, stopped)) {
        throw std::runtime_error("Reverse section did not complete on its configured control path");
    }
    costmap->cleanup();
    costmap.reset();
    rclcpp::shutdown();
}
''')
        build = tmp_path / 'build'
        configured = subprocess.run(['cmake', '-S', str(tmp_path), '-B', str(build)], capture_output=True, text=True)
        assert configured.returncode == 0, configured.stdout + configured.stderr
        compiled = subprocess.run(['cmake', '--build', str(build), '--parallel', '2'], capture_output=True, text=True)
        assert compiled.returncode == 0, compiled.stdout + compiled.stderr
        subprocess.run([str(build / 'ordered_probe')], check=True, timeout=15)


def test_navigation_uses_packaged_mppi_and_smac():
    import xml.etree.ElementTree as element_tree
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    for package_name in ('nav2_mppi_controller', 'nav2_smac_planner'):
        assert get_package_prefix(package_name) == '/opt/ros/humble'
    plugins = element_tree.parse(Path(get_package_share_directory('nav2_mppi_controller')) / 'mppic.xml')
    assert [plugin.attrib for plugin in plugins.findall('.//class')] == [
        {'type': 'nav2_mppi_controller::MPPIController', 'base_class_type': 'nav2_core::Controller'}]
    lattice_plugins = element_tree.parse(
        Path(get_package_share_directory('nav2_smac_planner')) / 'smac_plugin_lattice.xml')
    assert any(plugin.attrib['name'] == 'nav2_smac_planner/SmacPlannerLattice'
               for plugin in lattice_plugins.findall('.//class'))


@pytest.mark.parametrize('use_sim_time', [True, False])
@pytest.mark.parametrize('blocked_at', [None, 'ready', 'validate', 'server'])
def test_start_requires_ready_sensors_and_valid_route_on_both_clocks(use_sim_time, blocked_at):
    from rclpy.time import Time
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.node = Mock()
    executor.node.get_parameter.return_value.value = use_sim_time
    executor.node.get_clock.return_value.now.return_value = Time(nanoseconds=1_000_000_000)
    executor.busy = False
    executor.report = Mock()
    executor.ready = Mock(side_effect=ValueError('Laser scan is stale') if blocked_at == 'ready' else None)
    executor.validate = Mock(side_effect=ValueError('Unsafe route') if blocked_at == 'validate' else None)
    route = make_path([(1, 1, 0), (2, 1, 0)])
    executor.robot_pose = Mock(return_value=route.poses[0])
    executor.navigator = Mock()
    executor.navigator.server_is_ready.return_value = blocked_at != 'server'
    executor.follower = Mock()
    executor.pause_explore = Mock()
    executor.send = Mock()
    executor.start(route)
    executor.ready.assert_called_once()
    if blocked_at:
        assert not executor.busy
        executor.send.assert_not_called()
        executor.pause_explore.publish.assert_not_called()
        assert executor.report.call_args.args[0] == 'blocked'
    else:
        assert executor.busy
        executor.validate.assert_called_once_with(route, ingress=False)
        executor.send.assert_called_once()
        client, goal = executor.send.call_args.args
        assert client is executor.navigator
        assert goal.pose.pose == route.poses[0].pose
        assert goal.pose.header.frame_id == route.header.frame_id
        assert goal.behavior_tree == ''


@pytest.mark.parametrize('navigation', ['to_pose', 'through_poses'])
def test_ordinary_navigation_explicitly_selects_original_goal_checker(navigation):
    import xml.etree.ElementTree as element_tree
    root = Path(__file__).resolve().parents[1]
    with (root / 'config/coverage_sim.yaml').open() as stream:
        params = yaml.safe_load(stream)
    configured = params['bt_navigator']['ros__parameters'][f'default_nav_{navigation}_bt_xml']
    tree = element_tree.parse(root / 'behavior_trees' / Path(configured).name)
    installed = Path('/opt/ros/humble/share/nav2_bt_navigator/behavior_trees')
    original = element_tree.parse(installed / f'navigate_{navigation}_w_replanning_and_recovery.xml')
    if navigation == 'to_pose':
        reference = element_tree.parse(
            installed / 'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml')
        guard = tree.find('.//Fallback[@name="FallbackComputePathToPose"]')
        expected_guard = reference.find('.//Fallback[@name="FallbackComputePathToPose"]')
        assert guard is not None and expected_guard is not None
        assert [(node.tag, node.attrib) for node in guard.iter()] == [
            (node.tag, node.attrib) for node in expected_guard.iter()]
        planning = tree.find('.//RecoveryNode[@name="ComputePathToPose"]')
        compute = guard.find('ComputePathToPose')
        assert planning is not None and compute is not None
        planning.remove(guard)
        planning.insert(0, compute)
    assert len(list(tree.iter('FollowPath'))) == 1
    for follow in tree.iter('FollowPath'):
        assert follow.attrib.pop('goal_checker_id') == 'goal_checker'
        assert follow.attrib['controller_id'] == 'FollowPath'
    assert [(node.tag, node.attrib) for node in tree.iter()] == [
        (node.tag, node.attrib) for node in original.iter()]


@pytest.mark.parametrize('zone_size', [(8.0, 8.0), (2.7, 4.45)])
def test_real_coverage_server_returns_a_valid_forward_route(tmp_path, zone_size):
    from lifecycle_msgs.srv import ChangeState
    from opennav_coverage_msgs.action import ComputeCoveragePath
    from opennav_coverage_msgs.msg import Coordinate, Coordinates
    from rclpy.action import ActionClient
    import rclpy
    from frontier_explorer.coverage_path import FreeSpaceValidator, validate_forward_path
    from test_coverage_path import FOOTPRINT, make_grid

    root = Path(__file__).resolve().parents[1]
    with (root / 'config/coverage_sim.yaml').open() as stream:
        params = yaml.safe_load(stream)
    params = {'coverage_server': params['coverage_server']}
    params['coverage_server']['ros__parameters']['use_sim_time'] = False
    params['coverage_server']['ros__parameters']['coordinates_in_cartesian_frame'] = True
    params['coverage_server']['ros__parameters']['operation_width'] = 0.24
    config = tmp_path / 'params.yaml'
    config.write_text(yaml.safe_dump(params))
    log = (tmp_path / 'server.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'opennav_coverage', 'opennav_coverage',
                                '--ros-args', '--params-file', str(config)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('coverage_contract_test')
    try:
        lifecycle = node.create_client(ChangeState, '/coverage_server/change_state')
        assert lifecycle.wait_for_service(timeout_sec=10.0)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert await_future(node, lifecycle.call_async(request)).success
        client = ActionClient(node, ComputeCoveragePath, '/compute_coverage_path')
        assert client.wait_for_server(timeout_sec=5.0)
        goal = ComputeCoveragePath.Goal()
        goal.frame_id = 'map'
        goal.generate_headland = goal.generate_route = goal.generate_path = True
        width, height = zone_size
        zone = [(1.0, 1.0), (1.0 + width, 1.0), (1.0 + width, 1.0 + height),
            (1.0, 1.0 + height), (1.0, 1.0)]
        polygon = Coordinates()
        polygon.coordinates = [Coordinate(axis1=position_x, axis2=position_y) for position_x, position_y in zone]
        goal.polygons = [polygon]
        handle = await_future(node, client.send_goal_async(goal))
        assert handle.accepted
        wrapped = await_future(node, handle.get_result_async(), timeout=30.0)
        if width < 3.0:
            assert wrapped.status == GoalStatus.STATUS_ABORTED
            log.flush()
            assert 'Geometry does not contain point 0' in (tmp_path / 'server.log').read_text()
            return
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        result = wrapped.result
        assert result.error_code == 0
        assert result.coverage_path.contains_turns and result.coverage_path.swaths_ordered
        assert len(result.coverage_path.turns) == len(result.coverage_path.swaths) - 1
        assert validate_forward_path(result.nav_path, 0.20) > 5.0
        from frontier_explorer.coverage_path import path_from_swaths, work_paths
        work_path, sections = path_from_swaths(result.coverage_path)
        assert len(sections) == len(result.coverage_path.swaths)
        assert all(validate_forward_path(work, 0.20) > 0.05 for work in work_paths(work_path, sections))
        grid = make_grid()
        grid.info.width = grid.info.height = 220
        grid.data = [0] * (220 * 220)
        FreeSpaceValidator(grid, FOOTPRINT, 0.2, zone).check_path(result.nav_path)
    except Exception:
        log.flush()
        print((tmp_path / 'server.log').read_text())
        raise
    finally:
        import os
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('navigation', [None, 'to_pose', 'through_poses', 'to_pose_turn', 'to_pose_heading', 'to_pose_obstacle', 'to_pose_cancel', 'to_pose_cruise', 'to_pose_params', 'recorded_ingress', 'recorded_turn_ingress', 'recorded_turn_near_obstacle', 'closed_perimeter', 'directional_route', 'directional_finish', 'long_rows', 'coverage_preview', 'coverage_preview_motion', 'coverage_backup_recovery', 'coverage_transfers', 'coverage_transfers_deadband', 'coverage_parallel_rows', 'coverage_parallel_rows_deadband', 'coverage_parallel_rows_linear_deadband', 'coverage_parallel_rows_path_follow_handoff', 'coverage_parallel_rows_diagnostics', 'coverage_parallel_rows_search_window', 'to_pose_deadband', 'to_pose_turn_deadband', 'to_pose_heading_deadband', 'to_pose_obstacle_deadband', 'to_pose_cancel_deadband', 'to_pose_cruise_deadband', 'coverage_transfers_search_window', 'to_pose_turn_search_window', 'to_pose_heading_search_window', 'to_pose_obstacle_search_window', 'coverage_transfers_batch300', 'coverage_transfers_batch1000', 'coverage_transfers_work_batch1000'])
def test_installed_controller_and_ingress_on_synthetic_robot(tmp_path, navigation):
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose, NavigateThroughPoses
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav_msgs.msg import Path as PathMessage
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    import rclpy
    from frontier_explorer.coverage_path import pose_xy_yaw, validate_forward_path, work_paths
    from test_coverage_path import make_grid, make_path

    deadband_trial = navigation is not None and navigation.endswith('_deadband')
    linear_deadband_trial = navigation is not None and navigation.endswith('_linear_deadband')
    path_follow_handoff_trial = navigation is not None and navigation.endswith('_path_follow_handoff')
    search_window_trial = navigation is not None and navigation.endswith('_search_window')
    batch_sizes = {
        'coverage_transfers_batch300': (300, 300),
        'coverage_transfers_batch1000': (1000, 1000),
        'coverage_transfers_work_batch1000': (300, 1000),
    }.get(navigation)
    batch_trial = batch_sizes is not None
    diagnostic_trial = navigation in ('coverage_parallel_rows_diagnostics',
                                     'coverage_parallel_rows_search_window')
    if batch_trial:
        navigation = 'coverage_transfers'
    elif deadband_trial:
        navigation = navigation.removesuffix(
            '_linear_deadband' if linear_deadband_trial else '_deadband')
    elif path_follow_handoff_trial:
        navigation = navigation.removesuffix('_path_follow_handoff')
    elif search_window_trial:
        navigation = navigation.removesuffix('_search_window')
    elif diagnostic_trial:
        navigation = 'coverage_parallel_rows'
    ordinary_navigation = navigation == 'through_poses' or str(navigation).startswith('to_pose')
    managed_approach = navigation in ('coverage_backup_recovery', 'coverage_transfers',
                                      'coverage_parallel_rows', 'recorded_turn_near_obstacle',
                                      'directional_route')
    synthetic_sim_time = managed_approach
    replay_path = None
    replay_sections = None
    replay_map = None
    if navigation in ('recorded_turn_ingress', 'recorded_turn_near_obstacle'):
        turn = [(-0.1324164178, 2.8557980899, -2.7052601019),
                (-0.2035383976, 2.8059979793, -2.3561943213),
                (-0.2533385082, 2.7348759995, -2.0071285407),
                (-0.2758101272, 2.6510106418, -1.6580627600),
                (-0.2682425307, 2.5645169576, -1.3089965026)]
        if navigation == 'recorded_turn_near_obstacle':
            start_x, start_y, start_yaw = turn[0]
            center_x = start_x - 0.25 * math.sin(start_yaw)
            center_y = start_y + 0.25 * math.cos(start_yaw)
            turn.extend((center_x + 0.25 * math.sin(start_yaw + math.radians(angle)),
                         center_y - 0.25 * math.cos(start_yaw + math.radians(angle)),
                         start_yaw + math.radians(angle))
                        for angle in (100, 120, 140, 160, 180))
        position_x, position_y, yaw = turn[-1]
        turn.extend((position_x + distance * math.cos(yaw),
                     position_y + distance * math.sin(yaw), yaw)
                    for distance in [index * 0.025 for index in range(1, 61)])
        replay_path = make_path([(position_x + 4.0, position_y + 2.0, yaw)
                                 for position_x, position_y, yaw in turn])
    if navigation == 'recorded_ingress':
        import numpy as np
        with np.load(Path(__file__).parent / 'data/ingress_replay.npz', allow_pickle=False) as saved:
            replay_path = make_path(saved['poses'][28:])
            replay_map = make_grid()
            replay_map.info.height, replay_map.info.width = saved['occupancy'].shape
            replay_map.info.resolution = float(saved['resolution'])
            replay_map.data = saved['occupancy'].ravel().tolist()
            replay_map.info.origin = make_path([saved['origin']]).poses[0].pose
        for pose in replay_path.poses:
            pose.pose.position.x += 7.0
            pose.pose.position.y += 5.0

    root = Path(__file__).resolve().parents[1]
    spec = spec_from_file_location('navigation_launch', root / 'launch/navigation_launch.py')
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(use_sim_time=str(synthetic_sim_time).lower(), namespace='')
    params = module.CoverageTrialParameters(
        overlay=str(root / 'config/coverage_sim.yaml'),
        namespace=LaunchConfiguration('namespace'), source_file=str(root / 'config/explore.yaml'),
        param_rewrites={'use_sim_time': str(synthetic_sim_time).lower()}, convert_types=True)
    config = params.perform(context)
    with open(config) as stream:
        loaded = yaml.safe_load(stream)
    if batch_trial:
        controllers = loaded['controller_server']['ros__parameters']
        for controller_id, batch_size in zip(('FollowPath', 'CoverageFollowPath'), batch_sizes):
            controllers[controller_id]['batch_size'] = batch_size
        print(f'Coverage batch trial: ordinary={batch_sizes[0]}, work={batch_sizes[1]}', flush=True)
    if search_window_trial:
        ordinary = loaded['controller_server']['ros__parameters']['FollowPath']
        assert 'VelocityDeadbandCritic' not in ordinary['critics']
        ordinary['max_robot_pose_search_dist'] = 2.5
        print('Ordinary path search-window trial: 2.5 m, no deadband critic', flush=True)
    if diagnostic_trial:
        ordinary = loaded['controller_server']['ros__parameters']['FollowPath']
        ordinary['visualize'] = True
        ordinary['TrajectoryVisualizer'] = dict(trajectory_step=1000, time_step=59)
    if path_follow_handoff_trial:
        ordinary = loaded['controller_server']['ros__parameters']['FollowPath']
        assert 'VelocityDeadbandCritic' not in ordinary['critics']
        ordinary['PathFollowCritic']['threshold_to_consider'] = 0.25
        print('Ordinary PathFollow handoff trial: threshold 0.25 m, no deadband critic', flush=True)
    if deadband_trial:
        ordinary = loaded['controller_server']['ros__parameters']['FollowPath']
        ordinary['critics'].append('VelocityDeadbandCritic')
        wheel_linear_deadband = 0.35 * 0.0937
        ordinary['VelocityDeadbandCritic'] = dict(
            enabled=True, cost_power=1, cost_weight=35.0,
            deadband_velocities=[wheel_linear_deadband, 0.0,
                                 0.0 if linear_deadband_trial else
                                 2.0 * wheel_linear_deadband / 0.295])
    clearance = loaded['coverage_manager']['ros__parameters']['clearance']
    if ordinary_navigation or managed_approach:
        navigator = loaded['bt_navigator']['ros__parameters']
        navigator['wait_for_service_timeout'] = 5000
        for kind in ('to_pose', 'through_poses'):
            configured = navigator[f'default_nav_{kind}_bt_xml']
            navigator[f'default_nav_{kind}_bt_xml'] = str(root / 'behavior_trees' / Path(configured).name)
    if ordinary_navigation or managed_approach:
        config = str(tmp_path / 'navigation_params.yaml')
        Path(config).write_text(yaml.dump(loaded, Dumper=module.RosParametersDumper))
    logs, processes = [], []
    rclpy.init()
    node = rclpy.create_node('synthetic_robot')
    if synthetic_sim_time:
        from rclpy.clock import Clock, ClockType
        from rclpy.parameter import Parameter
        from rclpy.time import Time
        from rosgraph_msgs.msg import Clock as ClockMessage
        node.set_parameters([Parameter('use_sim_time', value=True)])
        clock_pub = node.create_publisher(ClockMessage, '/clock', 10)
        clock_started = time.monotonic_ns()

        def advance_clock():
            message = ClockMessage()
            message.clock = Time(nanoseconds=1_000_000_000 + time.monotonic_ns() - clock_started).to_msg()
            clock_pub.publish(message)

        node.create_timer(0.01, advance_clock, clock=Clock(clock_type=ClockType.STEADY_TIME))
    static_tf = StaticTransformBroadcaster(node)
    fixed = TransformStamped()
    fixed.header.frame_id, fixed.child_frame_id = 'map', 'odom'
    fixed.transform.rotation.w = 1.0
    static_tf.sendTransform(fixed)
    broadcaster = TransformBroadcaster(node)
    map_pub = node.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    path_pub = node.create_publisher(PathMessage, '/coverage/execution_path',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
    grid = make_grid()
    grid.info.width = grid.info.height = 200
    grid.data = [0] * 40000
    if navigation in ('coverage_preview', 'coverage_preview_motion'):
        for row in range(90, 110):
            for column in range(90, 110):
                grid.data[row * 200 + column] = 100
    if navigation == 'to_pose_obstacle':
        for row in range(34, 46):
            for column in range(75, 85):
                grid.data[row * 200 + column] = 100
    if navigation == 'recorded_turn_near_obstacle':
        for row in range(66, 70):
            for column in range(72, 82):
                grid.data[row * 200 + column] = 100
        from frontier_explorer.coverage_path import FreeSpaceValidator
        from test_coverage_path import FOOTPRINT
        FreeSpaceValidator(grid, FOOTPRINT, clearance).check_path(replay_path)
    if replay_map:
        grid = replay_map
        grid.info.origin.position.x += 7.0
        grid.info.origin.position.y += 5.0
    if navigation in ('directional_route', 'directional_finish'):
        from frontier_explorer.coverage_path import directional_coverage, FreeSpaceValidator
        from test_coverage_path import FOOTPRINT
        zone = [(1, 1), (3.7, 1), (3.7, 5.45), (1, 5.45)]
        replay_path, details = directional_coverage(zone, grid.header,
            FreeSpaceValidator(grid, FOOTPRINT, clearance, zone), 0.20)
        replay_sections = details['work_sections']
        if navigation == 'directional_finish':
            replay_path = work_paths(replay_path, replay_sections)[-1]
            replay_path.poses = replay_path.poses[-60:]
    grid.header.stamp = node.get_clock().now().to_msg()
    map_pub.publish(grid)
    position = [2.0, 2.0, 0.0]
    if navigation == 'to_pose_turn':
        position[2] = math.pi / 2.0
    if navigation == 'coverage_parallel_rows':
        position[:] = (3.0, 2.0, math.pi / 2.0)
    if replay_path:
        from frontier_explorer.coverage_path import pose_xy_yaw
        position[:] = pose_xy_yaw(replay_path.poses[0].pose)
        if navigation == 'recorded_ingress':
            position[0] += 0.024 * math.sin(position[2])
            position[1] -= 0.024 * math.cos(position[2])
            position[2] -= 0.012
        if navigation in ('recorded_turn_ingress', 'recorded_turn_near_obstacle'):
            position[:] = (3.8567104123, 4.8353081729, -2.6715236869)
    velocity = [0.0, 0.0]
    commands = []
    command_times = []
    raw_commands = []
    raw_roles = []
    coverage_owner = [None]
    plans = []
    node.create_subscription(PathMessage, '/plan', plans.append, 10)
    diagnostic_samples = []
    optimal_trajectory = []
    diagnostic_started = time.monotonic()
    if diagnostic_trial:
        from visualization_msgs.msg import MarkerArray

        def observe_trajectories(message):
            optimal_trajectory[:] = [(marker.pose.position.x, marker.pose.position.y)
                                     for marker in message.markers
                                     if marker.ns == 'Optimal Trajectory']

        def observe_local_path(message):
            owner = coverage_owner[0]
            if (owner is None or owner.section_index != 1 or owner.phase != 'navigating_ingress'
                    or not message.poses):
                return
            elapsed = time.monotonic() - diagnostic_started
            if diagnostic_samples and elapsed - diagnostic_samples[-1]['elapsed_s'] < 1.0:
                return
            points = [pose_xy_yaw(stamped.pose) for stamped in message.poses]
            endpoint_distance = math.dist(position[:2], points[-1][:2])
            ordinary = loaded['controller_server']['ros__parameters']['FollowPath']
            distance_gates = {
                critic: endpoint_distance < ordinary[critic]['threshold_to_consider']
                for critic in ('GoalCritic', 'GoalAngleCritic')}
            distance_gates.update({
                critic: endpoint_distance >= ordinary[critic]['threshold_to_consider']
                for critic in ('PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic')})
            diagnostic_samples.append(dict(
                elapsed_s=elapsed, position=tuple(position), frame=message.header.frame_id,
                local_path=points, local_endpoint_distance_m=endpoint_distance,
                local_length_m=sum(math.dist(first[:2], second[:2])
                                   for first, second in zip(points, points[1:])),
                critic_distance_gates=distance_gates,
                optimal_trajectory=list(optimal_trajectory), command=raw_commands[-1]
                    if raw_commands else None,
                limitations='Distance gates only; additional critic conditions and costs are not observed.'))

        node.create_subscription(MarkerArray, '/trajectories', observe_trajectories, 1)
        node.create_subscription(PathMessage, '/transformed_global_plan', observe_local_path, 1)
    observed_positions = []
    motion_metrics = dict(reverse_distance_m=0.0, reverse_episode_s=0.0,
                          longest_reverse_s=0.0)
    wheel_deadband = 0.0 if navigation == 'coverage_parallel_rows' else 0.35
    if ordinary_navigation or managed_approach:
        from frontier_explorer.coverage_path import FreeSpaceValidator
        from test_coverage_path import FOOTPRINT
        motion_validator = FreeSpaceValidator(grid, FOOTPRINT, clearance)
    def command(message):
        velocity[:] = [message.linear.x, message.angular.z]
        commands.append(tuple(velocity))
        command_times.append(time.monotonic())
        left = (velocity[0] - 0.295 * velocity[1] / 2.0) / 0.0937
        right = (velocity[0] + 0.295 * velocity[1] / 2.0) / 0.0937
        left = left if abs(left) >= wheel_deadband else 0.0
        right = right if abs(right) >= wheel_deadband else 0.0
        velocity[:] = [(left + right) * 0.0937 / 2.0, (right - left) * 0.0937 / 0.295]
    node.create_subscription(Twist, '/cmd_vel', command, 10)

    def raw_command(message):
        raw_commands.append((message.linear.x, message.angular.z))
        raw_roles.append(coverage_owner[0].phase if coverage_owner[0] is not None else None)

    node.create_subscription(Twist, '/cmd_vel_nav', raw_command, 10)
    last = [time.monotonic()]
    def update_pose():
        now = time.monotonic()
        elapsed = min(now - last[0], 0.1)
        last[0] = now
        if velocity[0] < -0.001:
            motion_metrics['reverse_distance_m'] -= velocity[0] * elapsed
            motion_metrics['reverse_episode_s'] += elapsed
            motion_metrics['longest_reverse_s'] = max(
                motion_metrics['longest_reverse_s'], motion_metrics['reverse_episode_s'])
        else:
            motion_metrics['reverse_episode_s'] = 0.0
        position[0] += velocity[0] * math.cos(position[2]) * elapsed
        position[1] += velocity[0] * math.sin(position[2]) * elapsed
        position[2] += velocity[1] * elapsed
        if ordinary_navigation or managed_approach:
            motion_validator.check_pose(*position)
            observed_positions.append(tuple(position))
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y = position[:2]
        transform.transform.rotation.z = math.sin(position[2] / 2)
        transform.transform.rotation.w = math.cos(position[2] / 2)
        broadcaster.sendTransform(transform)
        odom = Odometry()
        odom.header, odom.child_frame_id = transform.header, 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = position[:2]
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = velocity
        odom_pub.publish(odom)
    node.create_timer(0.02, update_pose)
    try:
        servers = [('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
                   ('nav2_velocity_smoother', 'velocity_smoother')]
        if ordinary_navigation or managed_approach:
            servers.extend([('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')])
        for package, executable in servers:
            if executable == 'bt_navigator':
                from nav2_msgs.action import BackUp, Spin, Wait
                for action_type, action_name in ((Spin, 'spin'), (BackUp, 'backup'), (Wait, 'wait'),
                                                 (ComputePathToPose, 'compute_path_to_pose'),
                                                 (FollowPath, 'follow_path')):
                    dependency = ActionClient(node, action_type, action_name)
                    assert dependency.wait_for_server(timeout_sec=5), action_name
                    dependency.destroy()
            log = (tmp_path / f'{executable}.log').open('w')
            logs.append(log)
            remappings = []
            if executable in ('controller_server', 'velocity_smoother'):
                remappings = ['-r', 'cmd_vel:=cmd_vel_nav']
            if executable == 'velocity_smoother':
                remappings.extend(['-r', 'cmd_vel_smoothed:=cmd_vel'])
            processes.append(subprocess.Popen(['ros2', 'run', package, executable, '--ros-args',
                                                '--params-file', config] + remappings, stdout=log,
                                               stderr=subprocess.STDOUT, start_new_session=True))
            client = node.create_client(ChangeState, f'/{executable}/change_state')
            assert client.wait_for_service(timeout_sec=10)
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                assert await_future(node, client.call_async(request), timeout=15).success
        if ordinary_navigation:
            if navigation == 'to_pose_params':
                from rcl_interfaces.srv import SetParameters
                from rclpy.parameter import Parameter
                parameters = node.create_client(SetParameters, '/controller_server/set_parameters')
                assert parameters.wait_for_service(timeout_sec=5)
                request = SetParameters.Request(parameters=[
                    Parameter('use_sim_time', value=False).to_parameter_msg()])
                response = await_future(node, parameters.call_async(request))
                assert response.results and all(result.successful for result in response.results)
                node.destroy_client(parameters)
            kind = 'through_poses' if navigation == 'through_poses' else 'to_pose'
            action_type = NavigateToPose if kind == 'to_pose' else NavigateThroughPoses
            client = ActionClient(node, action_type, f'navigate_{kind}')
            assert client.wait_for_server(timeout_sec=5)
            goal = action_type.Goal()
            poses = make_path([(2.5, 2, 0), (3, 2, 0)]).poses
            if navigation == 'to_pose_heading':
                poses = make_path([(3, 2, math.pi / 2.0)]).poses
            elif navigation in ('to_pose_obstacle', 'to_pose_cancel'):
                poses = make_path([(6, 2, 0)]).poses
            elif navigation == 'to_pose_cruise':
                poses = make_path([(8, 2, 0)]).poses
            for pose in poses:
                pose.header.frame_id = 'map'
                pose.header.stamp = node.get_clock().now().to_msg()
            if kind == 'to_pose':
                goal.pose = poses[-1]
            else:
                goal.poses = poses
            handle = await_future(node, client.send_goal_async(goal))
            assert handle.accepted
            result_future = handle.get_result_async()
            if navigation == 'to_pose_cancel':
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline and not any(linear > 0.05 for linear, _ in commands):
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert any(linear > 0.05 for linear, _ in commands)
                canceled = await_future(node, handle.cancel_goal_async())
                assert canceled.goals_canceling
                result = await_future(node, result_future)
                assert result.status == GoalStatus.STATUS_CANCELED
            else:
                try:
                    result = await_future(node, result_future, timeout=45)
                except AssertionError as exc:
                    raise AssertionError(f'Ordinary MPPI navigation timed out: pose={position}; '
                                         f'goal={pose_xy_yaw(poses[-1].pose)}; commands={commands[-10:]}') from exc
                assert result.status == GoalStatus.STATUS_SUCCEEDED
                target = pose_xy_yaw(poses[-1].pose)
                from frontier_explorer.coverage_path import angle_difference
                assert math.dist(position[:2], target[:2]) <= 0.25 + math.sqrt(2.0) * grid.info.resolution
                assert abs(angle_difference(position[2], target[2])) <= 0.25 + 0.01
            if navigation == 'to_pose_turn':
                assert any(abs(angular) > 0.2 and linear < 0.05 for linear, angular in commands)
            if navigation == 'to_pose_obstacle':
                assert max(abs(sample[1] - 2.0) for sample in observed_positions) > 0.50
            if navigation == 'to_pose_cruise':
                assert max(linear for linear, _ in commands) >= 0.35
            assert any(linear > 0.05 for linear, angular in commands)
            assert raw_commands
            maximum_linear = loaded['controller_server']['ros__parameters']['FollowPath']['vx_max']
            violations = [(linear, angular) for linear, angular in commands + raw_commands
                          if not math.isfinite(linear) or not math.isfinite(angular)
                          or linear < -0.001 or linear > maximum_linear + 0.01
                          or abs(angular) > 1.0 + 0.05]
            if deadband_trial:
                print(f'Deadband {navigation}: minimum linear command='
                      f'{min(linear for linear, angular in commands + raw_commands):.6f} m/s; '
                      f'measured motion={motion_metrics}', flush=True)
            assert not violations, f'Ordinary MPPI command envelope violations: {violations[:10]}'
            stop_deadline = time.monotonic() + 2.0
            while commands[-1] != (0.0, 0.0) and time.monotonic() < stop_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert commands[-1] == (0.0, 0.0)
            for log in logs:
                log.flush()
                assert 'FollowPath called with goal_checker name' not in Path(log.name).read_text()
            return
        if not managed_approach:
            planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
            assert planner.wait_for_server(timeout_sec=5)
            request = ComputePathToPose.Goal()
            request.start = make_path([tuple(position)]).poses[0]
            request.goal = make_path([(position[0] + 0.4 * math.cos(position[2]),
                           position[1] + 0.4 * math.sin(position[2]), position[2])]).poses[0]
            request.start.header.frame_id = request.goal.header.frame_id = 'map'
            request.use_start = True
            request.planner_id = 'GridBased'
            handle = await_future(node, planner.send_goal_async(request))
            assert handle.accepted
            planned = await_future(node, handle.get_result_async())
            assert planned.status == GoalStatus.STATUS_SUCCEEDED
            assert planned.result.path.poses
            assert planned.result.path.header.frame_id == 'map'
            for stamped in planned.result.path.poses:
                pose_xy_yaw(stamped.pose)
        if managed_approach:
            from sensor_msgs.msg import LaserScan
            from rclpy.executors import SingleThreadedExecutor
            from frontier_explorer.coverage_manager import CoverageManager
            manager = CoverageManager()
            if synthetic_sim_time:
                manager.set_parameters([Parameter('use_sim_time', value=True)])
            manager._clearance = clearance
            manager._execution.clearance = clearance
            scan_pub = node.create_publisher(LaserScan, '/scan', 10)

            def publish_inputs():
                grid.header.stamp = node.get_clock().now().to_msg()
                map_pub.publish(grid)
                scan = LaserScan()
                scan.header.frame_id = 'base_link'
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.angle_min = -math.pi
                scan.angle_max = math.pi
                scan.angle_increment = math.pi / 180.0
                scan.range_min = 0.05
                scan.range_max = 10.0
                scan.ranges = [float('inf')] * 361
                scan_pub.publish(scan)

            inputs_timer = node.create_timer(0.1, publish_inputs)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(manager)
            try:
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                coverage = manager._execution
                coverage_owner[0] = coverage
                coverage.report = Mock(wraps=coverage.report)
                coverage.ready()
                if navigation != 'coverage_backup_recovery':
                    coverage.recovery_enabled = False
                    points = [(2 + index * 0.025, 2.6, 0) for index in range(41)]
                    points.extend((3 - index * 0.025, 3.2, math.pi) for index in range(41))
                    points.extend((2 + index * 0.025, 3.8, 0) for index in range(41))
                    route = make_path(points)
                    route_sections = [[0, 40], [41, 81], [82, 122]]
                    if navigation == 'recorded_turn_near_obstacle':
                        route = PathMessage(header=replay_path.header, poses=replay_path.poses[-61:])
                        route_sections = None
                    elif navigation == 'directional_route':
                        route, route_sections = replay_path, replay_sections
                    elif navigation == 'coverage_parallel_rows':
                        row_length = 2.5908778585735166
                        row_points = 88
                        points = [(3.0, 2.0 + row_length * index / (row_points - 1),
                                   math.pi / 2.0) for index in range(row_points)]
                        points.extend((2.75, 2.0 + row_length * (1.0 - index / (row_points - 1)),
                                       -math.pi / 2.0) for index in range(row_points))
                        route = make_path(points)
                        route_sections = [[0, row_points - 1], [row_points, 2 * row_points - 1]]
                    for work in work_paths(route, route_sections):
                        validate_forward_path(work, 0.20)
                    coverage.start(route, work_sections=route_sections)
                else:
                    coverage.busy = True
                    coverage.recovery_backup_enabled = True
                    coverage.execution_path = make_path([(2, 2, 0), (2.5, 2, 0), (3, 2, 0)])
                    coverage.cursor = 1
                    coverage.phase = 'following'
                    coverage.request_id = 1
                    coverage.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_ABORTED)), 1)
                    assert coverage.phase == 'backing_up'
                switches = []
                previous_section = coverage.section_index
                phases = [coverage.phase]
                phase_started = time.monotonic()
                phase_times = [(coverage.phase, 0.0)]
                work_handoffs = []
                coverage_command_start = None
                handoff_motion = None
                deadline = time.monotonic() + (600.0 if navigation == 'directional_route' else 90.0)
                while coverage.busy and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                    if phases[-1] != coverage.phase:
                        phases.append(coverage.phase)
                        phase_times.append((coverage.phase, time.monotonic() - phase_started))
                        if coverage.phase == 'following':
                            start = coverage.sections[coverage.section_index][0]
                            work_handoffs.append(dict(section=coverage.section_index, position=tuple(position),
                                target=pose_xy_yaw(coverage.route.poses[start].pose)))
                    if coverage.phase == 'following' and coverage_command_start is None:
                        coverage_command_start = len(raw_commands)
                        handoff_motion = (coverage.motion_linear, coverage.motion_angular)
                    if coverage.section_index != previous_section:
                        switches.append((coverage.motion_linear, coverage.motion_angular))
                        previous_section = coverage.section_index
                if diagnostic_trial:
                    import json
                    for sample in diagnostic_samples:
                        print('MPPI_LOCAL_DIAGNOSTIC ' + json.dumps(sample), flush=True)
                    assert diagnostic_samples, 'No ordinary MPPI transformed-path diagnostics received'
                    assert any(sample['optimal_trajectory'] for sample in diagnostic_samples), (
                        'No ordinary MPPI optimal trajectory received')
                if coverage.phase != 'completed':
                    pytest.fail(str(dict(
                        phase=coverage.phase, position=position, section=coverage.section_index,
                        endpoint=pose_xy_yaw((coverage.execution_path.poses[
                            coverage.sections[coverage.section_index][1]] if coverage.execution_path
                            else coverage.route.poses[0]).pose), phases=phases,
                        phase_times=phase_times,
                        ingress_target=pose_xy_yaw(coverage.route.poses[
                            coverage.sections[coverage.section_index][0]].pose),
                        planner_endpoint=pose_xy_yaw(plans[-1].poses[-1].pose)
                            if plans and plans[-1].poses else None,
                        planner_path_length_m=sum(math.dist(
                            pose_xy_yaw(first.pose)[:2], pose_xy_yaw(second.pose)[:2])
                            for first, second in zip(plans[-1].poses, plans[-1].poses[1:]))
                            if plans else None,
                        measured_motion=tuple(velocity),
                        raw_tail=raw_commands[-5:], cursor=coverage.cursor,
                        last_status=coverage.report.call_args, work_handoffs=work_handoffs,
                        section_progress=getattr(coverage.section_tracker, 'progress', None),
                        section_length=getattr(coverage.section_tracker, 'length', None))))
                assert 'navigating_ingress' in phases
                assert 'waiting_ingress_stop' in phases
                assert coverage_command_start is not None
                assert handoff_motion[0] <= 0.02 and handoff_motion[1] <= 0.05
                coverage_commands = [command for command, role in zip(raw_commands, raw_roles)
                                     if role == 'following']
                assert coverage.recovery_attempts == (1 if navigation == 'coverage_backup_recovery' else 0)
                if navigation == 'coverage_backup_recovery':
                    assert any(linear < -0.03 for linear, angular in commands)
                assert any(linear > 0.10 for linear, angular in commands)
                if deadband_trial or path_follow_handoff_trial or search_window_trial or batch_trial:
                    transit_commands = [
                        command for command, role in zip(raw_commands, raw_roles)
                        if role == 'navigating_ingress']
                    maximum_linear = loaded['controller_server']['ros__parameters']['FollowPath']['vx_max']
                    violations = [
                        (linear, angular) for linear, angular in transit_commands
                        if not math.isfinite(linear) or not math.isfinite(angular)
                        or linear < -0.001 or linear > maximum_linear + 0.01
                        or abs(angular) > 1.0 + 0.05]
                    print(
                        f'Coverage controller trial completed {len(route_sections)} sections; '
                        f'phase times={phase_times}; final pose={position}; '
                        f'measured motion={motion_metrics}', flush=True)
                if navigation in ('coverage_transfers', 'coverage_parallel_rows', 'directional_route'):
                    assert switches
                    assert all(linear <= 0.02 and angular <= 0.05 for linear, angular in switches)
                    assert 'waiting_work_stop' in phases
                    assert phases.count('navigating_ingress') == len(route_sections)
                    assert all(math.isfinite(linear) and math.isfinite(angular)
                               and -1e-6 <= linear <= 0.20 + 0.005
                               for linear, angular in coverage_commands), coverage_commands[-10:]
                    assert all(math.isfinite(linear) and math.isfinite(angular)
                               and abs(linear) <= 0.20 + 0.005 and abs(angular) <= abs(linear) / 0.20 + 1e-6
                               for linear, angular in coverage_commands), coverage_commands[-10:]
                    assert math.dist(position[:2], pose_xy_yaw(route.poses[-1].pose)[:2]) < 0.10
                    assert observed_positions
                elif navigation == 'coverage_backup_recovery':
                    assert all(linear >= -1e-6 for linear, angular in raw_commands)
                else:
                    assert math.dist(position[:2], pose_xy_yaw(coverage.route.poses[-1].pose)[:2]) < 0.10
                if navigation not in ('directional_route', 'coverage_parallel_rows'):
                    assert position[0] > 2.9
                stop_deadline = time.monotonic() + 2.0
                while commands[-1] != (0.0, 0.0) and time.monotonic() < stop_deadline:
                    executor.spin_once(timeout_sec=0.02)
                assert commands[-1] == (0.0, 0.0)
                if deadband_trial or path_follow_handoff_trial or search_window_trial or batch_trial:
                    assert not violations, (
                        f'Ordinary transit command envelope violations: {violations[:10]}')
            finally:
                if manager._execution.busy:
                    manager._execution.cancel()
                    deadline = time.monotonic() + 5.0
                    while manager._execution.busy and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.02)
                node.destroy_timer(inputs_timer)
                executor.remove_node(manager)
                executor.remove_node(node)
                manager.destroy_node()
                executor.shutdown()
            return
        if navigation in ('coverage_preview', 'coverage_preview_motion'):
            from frontier_explorer.coverage_manager import CoverageManager
            from frontier_explorer.coverage_path import FreeSpaceValidator
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.parameter import Parameter
            from test_coverage_path import FOOTPRINT
            manager = CoverageManager()
            manager._publish_status = Mock(wraps=manager._publish_status)
            manager.set_parameters([Parameter('layout_mode', value='directional')])
            manager._clearance = clearance
            manager._execution.clearance = clearance
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(manager)
            try:
                map_timer = node.create_timer(0.5, lambda: map_pub.publish(grid))
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                manager._start_preview()
                deadline = time.monotonic() + 125.0
                while manager._busy() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                assert manager._state == 'preview_ready', manager._publish_status.call_args
                assert manager._last_preview_valid
                assert manager._validated_grid is not None
                validator = FreeSpaceValidator(grid, FOOTPRINT, clearance)
                paths = work_paths(manager._cached_path, manager._cached_preview_details['work_sections'])
                for work in paths:
                    validator.check_path(work)
                assert sum(validate_forward_path(work, 0.20) for work in paths) > 150.0
                assert len(manager._display_path.poses) < len(manager._cached_path.poses)
                assert not commands, 'Preview must never move the synthetic robot'
                if navigation == 'coverage_preview_motion':
                    from copy import deepcopy
                    from frontier_explorer.coverage_path import angle_difference
                    replay_path = deepcopy(next(work for work in paths
                        if any(abs(angle_difference(pose_xy_yaw(second.pose)[2], pose_xy_yaw(first.pose)[2])) > 0.01
                               for first, second in zip(work.poses, work.poses[1:]))))
                    position[:] = pose_xy_yaw(replay_path.poses[0].pose)
                node.destroy_timer(map_timer)
            finally:
                executor.remove_node(manager)
                executor.remove_node(node)
                manager.destroy_node()
                executor.shutdown()
            if navigation == 'coverage_preview':
                return
            update_pose()
            settle_deadline = time.monotonic() + 0.5
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
        row_length = 6.0 if navigation == 'long_rows' else 0.6
        row_steps = round(row_length / 0.025)
        points = [(2 + index * 0.025, 2, 0) for index in range(row_steps + 1)]
        points.extend((2 + row_length + 0.3 * math.sin(angle), 2.3 - 0.3 * math.cos(angle), angle)
                      for angle in [index * math.pi / 60 for index in range(1, 61)])
        points.extend((2 + row_length - index * 0.025, 2.6, math.pi) for index in range(1, row_steps + 1))
        if navigation == 'long_rows':
            replay_path = make_path(points)
        if navigation == 'closed_perimeter':
            points = [(2 + 0.4 * math.sin(angle), 2.4 - 0.4 * math.cos(angle), angle)
                      for angle in [index * math.pi / 60 for index in range(121)]]
            replay_path = make_path(points)
        follower = ActionClient(node, FollowPath, 'follow_path')
        assert follower.wait_for_server(timeout_sec=5)
        goal = FollowPath.Goal()
        goal.path = replay_path if replay_path else make_path(points)
        goal.controller_id, goal.goal_checker_id = 'CoverageFollowPath', 'coverage_goal_checker'
        path_pub.publish(goal.path)
        handle = await_future(node, follower.send_goal_async(goal))
        assert handle.accepted
        motion_started = time.monotonic()
        if replay_path:
            from frontier_explorer.navigation_metrics import OrderedPathTracker
            tracker = OrderedPathTracker([(pose.pose.position.x, pose.pose.position.y)
                                           for pose in replay_path.poses], search_distance=0.6)
            result = handle.get_result_async()
            full_route = navigation in ('directional_route', 'directional_finish', 'long_rows', 'closed_perimeter', 'recorded_turn_ingress', 'recorded_turn_near_obstacle', 'coverage_preview_motion')
            deadline = time.monotonic() + (600.0 if navigation in ('directional_route', 'long_rows') else 40.0)
            target_progress = tracker.length if full_route else 1.8
            last_report = time.monotonic()
            deviations = []
            straight_deviations = []
            while time.monotonic() < deadline and not result.done() and (full_route or tracker.progress < target_progress):
                rclpy.spin_once(node, timeout_sec=0.02)
                tracking = tracker.update(*position)
                if tracking:
                    deviations.append(tracking['path_distance_m'])
                    if navigation == 'long_rows' and (
                            0.5 < tracker.progress < 5.5 or 7.5 < tracker.progress < 12.4):
                        straight_deviations.append(tracking['path_distance_m'])
                if time.monotonic() - last_report > 20.0:
                    print(f'Ordered progress {tracker.progress:.2f}/{tracker.length:.2f} m', flush=True)
                    last_report = time.monotonic()
            progressed = tracker.progress
            completed = result.done() and result.result().status == GoalStatus.STATUS_SUCCEEDED
            if not result.done():
                await_future(node, handle.cancel_goal_async())
                await_future(node, result)
            if full_route:
                assert completed, (
                    f'Route stopped at {progressed:.3f}/{tracker.length:.3f} m; '
                    f'pose={position}; goal={pose_xy_yaw(replay_path.poses[-1].pose)}; '
                    f'last commands={commands[-10:]}')
                assert tracker.length - progressed < 0.12
            else:
                assert progressed >= 1.8, f'Recorded ingress stalled at {progressed:.3f} m'
            assert max(deviations) < 0.15
            if navigation == 'long_rows':
                import numpy as np
                assert straight_deviations
                straight_p95 = np.quantile(straight_deviations, 0.95)
                print(f'Settled straight tracking p95 {straight_p95:.4f} m', flush=True)
                assert straight_p95 < 0.05
        else:
            followed = await_future(node, handle.get_result_async(), timeout=35)
            assert followed.status == GoalStatus.STATUS_SUCCEEDED
        assert len(commands) > 50
        assert len(raw_commands) > 50
        violations = [(linear, angular) for linear, angular in commands + raw_commands
                      if not math.isfinite(linear) or not math.isfinite(angular)
                      or linear < -1e-6 or linear > 0.20 + 0.005 or abs(angular) > linear / 0.20 + 1e-6]
        assert not violations, f'Controller command envelope violations: {violations[:10]}'
        assert any(abs(angular) > 0.2 for _, angular in commands)
        assert_useful_wheel_commands(commands, command_times, motion_started)
        stop_deadline = time.monotonic() + 2.0
        while commands[-1] != (0.0, 0.0) and time.monotonic() < stop_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert commands[-1] == (0.0, 0.0), 'Completion or cancellation did not stop the robot'
    except (Exception, pytest.fail.Exception):
        for log in logs:
            log.flush()
            print(Path(log.name).read_text()[-12000:])
        raise
    finally:
        for process in processes:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


def assert_useful_wheel_commands(commands, command_times, motion_started):
    import numpy as np
    assert len(commands) == len(command_times)
    settled_start = motion_started + 1.0
    settled_end = command_times[-1] - 1.0
    intervals = [(command, min(end, settled_end) - max(start, settled_start))
                 for command, start, end in zip(commands, command_times, command_times[1:])
                 if min(end, settled_end) > max(start, settled_start)]
    settled = [command for command, duration in intervals]
    assert settled
    slowest_wheel = [(linear - 0.295 * abs(angular) / 2.0) / 0.0937 for linear, angular in settled]
    turns = [linear for linear, angular in settled if abs(angular) >= 0.15]
    coast_duration = 0.0
    coast_episode = 0.0
    longest_coast = 0.0
    for speed, (_, duration) in zip(slowest_wheel, intervals):
        if speed < 0.35:
            coast_duration += duration
            coast_episode += duration
            longest_coast = max(longest_coast, coast_episode)
        else:
            coast_episode = 0.0
    coasting_fraction = coast_duration / sum(duration for command, duration in intervals)
    assert turns
    print(f'Settled turn median {np.median(turns):.3f} m/s; inner-wheel coast fraction '
          f'{coasting_fraction:.3%}; longest coast {longest_coast:.3f} s; '
          f'wheel p05 {np.quantile(slowest_wheel, 0.05):.3f} rad/s', flush=True)
    assert coasting_fraction < 0.05
    assert longest_coast < 0.5, f'Continuous below-deadband command for {longest_coast:.3f} s'
    assert np.median(turns) >= 0.13


def test_wheel_speed_gate_rejects_sustained_coasting_despite_low_fraction():
    commands = [(0.18, 0.5), (0.02, 0.15), (0.18, 0.5), (0.18, 0.5), (0.0, 0.0)]
    with pytest.raises(AssertionError, match='Continuous below-deadband'):
        assert_useful_wheel_commands(commands, [0.0, 1.1, 1.7, 20.0, 21.0], 0.0)


def test_wheel_speed_gate_excludes_startup_and_stopping():
    commands = [(0.0, 0.0), (0.18, 0.5), (0.0, 0.0), (0.0, 0.0)]
    assert_useful_wheel_commands(commands, [0.0, 1.0, 20.0, 21.0], 0.0)