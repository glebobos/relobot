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


def test_pending_cancel_is_sent_when_goal_is_accepted():
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.busy = True
    executor.handle = None
    executor.stop_state = None
    executor.cancel_sent = False
    executor.request_id = 1
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
    executor.tracker = SimpleNamespace(length=20.0, progress=3.0)
    executor.cursor = 0
    executor.report = Mock()
    executor.plan_pub = Mock()
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), 1)
    assert executor.phase == 'blocked'


def test_ingress_submits_one_complete_path_without_waypoint_conversion():
    from copy import deepcopy
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.request_id = 1
    executor.handle = Mock()
    executor.stop_state = None
    executor.phase = 'planning_ingress'
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
    ingress = make_path([(1, 1, 0), (1.5, 1, 0), (2, 1, 0)])
    executor.result(Mock(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                       result=SimpleNamespace(path=ingress))), 1)
    executor.send.assert_called_once()
    sent_goal = executor.send.call_args.args[1]
    assert sent_goal.controller_id == 'CoverageFollowPath'
    assert sent_goal.goal_checker_id == 'coverage_goal_checker'
    assert sent_goal.path.poses == ingress.poses + route_before.poses
    assert executor.route == route_before
    executor.validate.assert_called_once_with(sent_goal.path, ingress=True)


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
    assert controller['CoverageFollowPath']['VelocityDeadbandCritic']['deadband_velocities'] == [0.18, 0.0, 0.0]
    assert loaded['coverage_manager']['ros__parameters']['path_type'] == 'DUBIN'
    assert loaded['coverage_manager']['ros__parameters']['clearance'] == 0.25
    assert controller['FollowPath']['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'


@pytest.mark.parametrize('use_sim_time', [True, False])
@pytest.mark.parametrize('blocked_at', [None, 'ready', 'validate', 'server'])
def test_start_requires_ready_sensors_and_valid_route_on_both_clocks(use_sim_time, blocked_at):
    from test_coverage_path import make_path
    executor = CoverageExecution.__new__(CoverageExecution)
    executor.node = Mock()
    executor.node.get_parameter.return_value.value = use_sim_time
    executor.node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
    executor.busy = False
    executor.report = Mock()
    executor.ready = Mock(side_effect=ValueError('Laser scan is stale') if blocked_at == 'ready' else None)
    executor.validate = Mock(side_effect=ValueError('Unsafe route') if blocked_at == 'validate' else None)
    route = make_path([(1, 1, 0), (2, 1, 0)])
    executor.robot_pose = Mock(return_value=route.poses[0])
    executor.planner = Mock()
    executor.planner.server_is_ready.return_value = blocked_at != 'server'
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
        assert client is executor.planner
        assert goal.planner_id == 'CoverageIngress'
        assert goal.use_start


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


@pytest.mark.parametrize('navigation', [None, 'to_pose', 'through_poses', 'recorded_ingress', 'closed_perimeter', 'directional_route', 'directional_finish', 'long_rows'])
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
    from frontier_explorer.coverage_path import pose_xy_yaw, validate_forward_path
    from test_coverage_path import make_grid, make_path

    ordinary_navigation = navigation in ('to_pose', 'through_poses')
    replay_path = None
    replay_map = None
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
    context.launch_configurations.update(use_sim_time='false', namespace='')
    params = module.CoverageTrialParameters(
        overlay=str(root / 'config/coverage_sim.yaml'),
        namespace=LaunchConfiguration('namespace'), source_file=str(root / 'config/explore.yaml'),
        param_rewrites={'use_sim_time': 'false'}, convert_types=True)
    config = params.perform(context)
    with open(config) as stream:
        loaded = yaml.safe_load(stream)
    clearance = loaded['coverage_manager']['ros__parameters']['clearance']
    if ordinary_navigation:
        navigator = loaded['bt_navigator']['ros__parameters']
        for kind in ('to_pose', 'through_poses'):
            configured = navigator[f'default_nav_{kind}_bt_xml']
            navigator[f'default_nav_{kind}_bt_xml'] = str(root / 'behavior_trees' / Path(configured).name)
        config = str(tmp_path / 'navigation_params.yaml')
        Path(config).write_text(yaml.safe_dump(loaded))
    logs, processes = [], []
    rclpy.init()
    node = rclpy.create_node('synthetic_robot')
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
        if navigation == 'directional_finish':
            replay_path.poses = replay_path.poses[-60:]
    grid.header.stamp = node.get_clock().now().to_msg()
    map_pub.publish(grid)
    position = [2.0, 2.0, 0.0]
    if replay_path:
        from frontier_explorer.coverage_path import pose_xy_yaw
        position[:] = pose_xy_yaw(replay_path.poses[0].pose)
        if navigation == 'recorded_ingress':
            position[0] += 0.024 * math.sin(position[2])
            position[1] -= 0.024 * math.cos(position[2])
            position[2] -= 0.012
    velocity = [0.0, 0.0]
    commands = []
    command_times = []
    raw_commands = []
    def command(message):
        velocity[:] = [message.linear.x, message.angular.z]
        commands.append(tuple(velocity))
        command_times.append(time.monotonic())
        if not ordinary_navigation:
            left = (velocity[0] - 0.295 * velocity[1] / 2.0) / 0.0937
            right = (velocity[0] + 0.295 * velocity[1] / 2.0) / 0.0937
            left = left if abs(left) >= 0.35 else 0.0
            right = right if abs(right) >= 0.35 else 0.0
            velocity[:] = [(left + right) * 0.0937 / 2.0, (right - left) * 0.0937 / 0.295]
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(Twist, '/cmd_vel_nav',
                             lambda msg: raw_commands.append((msg.linear.x, msg.angular.z)), 10)
    last = [time.monotonic()]
    def update_pose():
        now = time.monotonic()
        elapsed = min(now - last[0], 0.1)
        last[0] = now
        position[0] += velocity[0] * math.cos(position[2]) * elapsed
        position[1] += velocity[0] * math.sin(position[2]) * elapsed
        position[2] += velocity[1] * elapsed
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
        if ordinary_navigation:
            servers.extend([('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')])
        for package, executable in servers:
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
            action_type = NavigateToPose if navigation == 'to_pose' else NavigateThroughPoses
            client = ActionClient(node, action_type, f'navigate_{navigation}')
            assert client.wait_for_server(timeout_sec=5)
            goal = action_type.Goal()
            poses = make_path([(2.5, 2, 0), (3, 2, 0)]).poses
            for pose in poses:
                pose.header.frame_id = 'map'
                pose.header.stamp = node.get_clock().now().to_msg()
            if navigation == 'to_pose':
                goal.pose = poses[-1]
            else:
                goal.poses = poses
            handle = await_future(node, client.send_goal_async(goal))
            assert handle.accepted
            result = await_future(node, handle.get_result_async(), timeout=20)
            assert result.status == GoalStatus.STATUS_SUCCEEDED
            assert position[0] > 2.6
            assert any(linear > 0.05 for linear, angular in commands)
            for log in logs:
                log.flush()
                assert 'FollowPath called with goal_checker name' not in Path(log.name).read_text()
            return
        planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        assert planner.wait_for_server(timeout_sec=5)
        request = ComputePathToPose.Goal()
        request.start = make_path([tuple(position)]).poses[0]
        request.goal = make_path([(position[0] + 0.4 * math.cos(position[2]),
                       position[1] + 0.4 * math.sin(position[2]), position[2])]).poses[0]
        request.start.header.frame_id = request.goal.header.frame_id = 'map'
        request.use_start = True
        request.planner_id = 'CoverageIngress'
        handle = await_future(node, planner.send_goal_async(request))
        assert handle.accepted
        planned = await_future(node, handle.get_result_async())
        assert planned.status == GoalStatus.STATUS_SUCCEEDED
        validate_forward_path(planned.result.path, 0.20)
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
            full_route = navigation in ('directional_route', 'directional_finish', 'long_rows', 'closed_perimeter')
            deadline = time.monotonic() + (600.0 if navigation in ('directional_route', 'long_rows') else 40.0)
            target_progress = tracker.length if full_route else 1.8
            last_report = time.monotonic()
            deviations = []
            straight_deviations = []
            while time.monotonic() < deadline and not result.done() and tracker.progress < target_progress:
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
                      or linear < -1e-6 or linear > 0.20 + 1e-6 or abs(angular) > linear / 0.20 + 1e-6]
        assert not violations, f'Controller command envelope violations: {violations[:10]}'
        assert any(abs(angular) > 0.2 for _, angular in commands)
        assert_useful_wheel_commands(commands, command_times, motion_started)
        stop_deadline = time.monotonic() + 2.0
        while commands[-1] != (0.0, 0.0) and time.monotonic() < stop_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert commands[-1] == (0.0, 0.0), 'Completion or cancellation did not stop the robot'
    except Exception:
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
    assert np.median(turns) >= 0.14


def test_wheel_speed_gate_rejects_sustained_coasting_despite_low_fraction():
    commands = [(0.18, 0.5), (0.02, 0.15), (0.18, 0.5), (0.18, 0.5), (0.0, 0.0)]
    with pytest.raises(AssertionError, match='Continuous below-deadband'):
        assert_useful_wheel_commands(commands, [0.0, 1.1, 1.7, 20.0, 21.0], 0.0)


def test_wheel_speed_gate_excludes_startup_and_stopping():
    commands = [(0.0, 0.0), (0.18, 0.5), (0.0, 0.0), (0.0, 0.0)]
    assert_useful_wheel_commands(commands, [0.0, 1.0, 20.0, 21.0], 0.0)