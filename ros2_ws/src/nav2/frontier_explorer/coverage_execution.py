"""Ordinary navigation approach followed by coverage with bounded stop guards."""

from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import math
import time
from uuid import uuid4

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID

from frontier_explorer.coverage_path import (
    FreeSpaceValidator, pose_xy_yaw, work_paths,
)
from frontier_explorer.navigation_metrics import OrderedPathTracker


class CoverageExecution:
    def __init__(self, node, report, validate, footprint, clearance, min_radius,
                 recovery_enabled=False,
                 recovery_max_attempts=2, recovery_backup_distance=0.15,
                 recovery_backup_speed=0.05, recovery_backup_enabled=False,
                 motion_odom_topic='/odometry/filtered'):
        self.node, self.report, self.validate = node, report, validate
        self.footprint, self.clearance, self.min_radius = footprint, clearance, min_radius
        if (not 0 <= recovery_max_attempts <= 3
                or not math.isfinite(recovery_backup_distance) or not 0.0 < recovery_backup_distance <= 0.30
                or not math.isfinite(recovery_backup_speed) or not 0.0 < recovery_backup_speed <= 0.10):
            raise ValueError('Coverage recovery limits are invalid')
        self.recovery_enabled = recovery_enabled
        self.recovery_backup_enabled = recovery_backup_enabled
        self.recovery_max_attempts = recovery_max_attempts
        self.recovery_backup_distance = recovery_backup_distance
        self.recovery_backup_speed = recovery_backup_speed
        self.recovery_attempts = 0
        self.recovery_route = None
        self.backup_start = None
        self.navigator = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.ingress_goal_id = None
        self.follower = ActionClient(node, FollowPath, 'follow_path')
        self.backup = ActionClient(node, BackUp, 'backup')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.plan_pub = node.create_publisher(Path, '/coverage/execution_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.control_plan_pub = node.create_publisher(Path, '/coverage/control_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pause_explore = node.create_publisher(Bool, '/explore/resume', 1)
        self.busy = False
        self.handle = None
        self.phase = 'idle'
        self.stop_state = None
        self.route = None
        self.execution_path = None
        self.tracker = None
        self.sections = []
        self.work_sections = None
        self.section_lengths = []
        self.section_index = 0
        self.section_tracker = None
        self.section_offset = 0.0
        self.pending_section_index = None
        self.last_motion_direction = None
        self.motion_stamp = Time()
        self.motion_received = 0.0
        self.motion_linear = math.inf
        self.motion_angular = math.inf
        self.stopped_since = None
        self.cursor = 0
        self.last_feedback = 0.0
        self.last_tracking_warning = 0.0
        self.coverage_start_index = 0
        self.local = None
        self.local_grid = None
        self.local_config = None
        self.local_received = 0.0
        self.scan_received = 0.0
        self.scan_stamp = Time()
        self.last_ros_time = None
        self.other_navigation = {}
        self.started = 0.0
        self.cancel_sent = False
        self.request_id = 0
        self.join_pool = ThreadPoolExecutor(max_workers=1)
        self.join_future = None
        latest_sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_costmap,
                     latest_sensor_qos)
        node.create_subscription(LaserScan, '/scan', self.on_scan, latest_sensor_qos)
        node.create_subscription(Odometry, motion_odom_topic, self.on_odometry, latest_sensor_qos)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for action in ('navigate_to_pose', 'navigate_through_poses'):
            node.create_subscription(GoalStatusArray, f'/{action}/_action/status',
                                     lambda msg, name=action: self.on_navigation(name, msg), qos)
        self.timer = node.create_timer(0.2, self.guard, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_navigation(self, name, message):
        self.other_navigation[name] = any(
            item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                            GoalStatus.STATUS_CANCELING)
            and not (name == 'navigate_to_pose'
                     and bytes(item.goal_info.goal_id.uuid) == getattr(self, 'ingress_goal_id', None))
            for item in message.status_list)
        if self.busy and self.other_navigation[name]:
            self.cancel('blocked', 'Another navigation action became active; coverage interrupted.')

    def on_scan(self, message):
        self.scan_received = time.monotonic()
        self.scan_stamp = Time.from_msg(message.header.stamp)

    def on_odometry(self, message):
        self.motion_stamp = Time.from_msg(message.header.stamp)
        self.motion_received = time.monotonic()
        self.motion_linear = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        self.motion_angular = abs(message.twist.twist.angular.z)

    def on_costmap(self, message):
        received = time.monotonic()
        previous = getattr(self, 'local_grid', None)
        config = (self.footprint, self.clearance)
        unchanged = (self.local is not None and previous is not None
                     and getattr(self, 'local_config', None) == config
                     and previous.header.frame_id == message.header.frame_id
                     and previous.info.width == message.info.width
                     and previous.info.height == message.info.height
                     and previous.info.resolution == message.info.resolution
                     and previous.info.origin == message.info.origin
                     and previous.data == message.data)
        try:
            if not unchanged:
                self.local = FreeSpaceValidator(message, self.footprint, self.clearance,
                                                occupied_threshold=100)
                self.local_config = (deepcopy(self.footprint), self.clearance)
            self.local_grid = message
            self.local_stamp = Time.from_msg(message.header.stamp)
            self.local_received = received
        except ValueError:
            self.local = None
            self.local_grid = None
            self.local_config = None

    def robot_pose(self, frame):
        stamped = self.tf.lookup_transform(frame, 'base_link', Time())
        age = (self.node.get_clock().now().nanoseconds - Time.from_msg(stamped.header.stamp).nanoseconds) / 1e9
        if not -0.1 <= age <= 0.5:
            raise ValueError('Robot transform is stale')
        result = PoseStamped()
        result.header.frame_id = frame
        result.header.stamp = stamped.header.stamp
        result.pose.position.x = stamped.transform.translation.x
        result.pose.position.y = stamped.transform.translation.y
        result.pose.orientation = stamped.transform.rotation
        return result

    def ready(self):
        if self.local is None or time.monotonic() - self.local_received > 1.0:
            raise ValueError('Waiting for a fresh full local costmap')
        if time.monotonic() - self.scan_received > 1.0:
            raise ValueError('Laser scan is stale')
        age = (self.node.get_clock().now().nanoseconds - self.scan_stamp.nanoseconds) / 1e9
        if not -0.1 <= age <= 1.0:
            raise ValueError(f'Laser scan timestamp is stale (ROS age={age:.3f} s, '
                             f'reception age={time.monotonic() - self.scan_received:.3f} s)')
        age = (self.node.get_clock().now().nanoseconds - self.local_stamp.nanoseconds) / 1e9
        if not -0.1 <= age <= 1.0:
            raise ValueError('Local costmap timestamp is stale')
        if any(self.other_navigation.values()):
            raise ValueError('Stop exploration and other navigation before starting coverage')

    def start(self, route, resuming=False, work_sections=None):
        if self.busy:
            self.report('busy', 'Coverage is still handling an action.')
            return
        try:
            self.ready()
            paths = work_paths(route, work_sections)
            if len(route.poses) <= 5000:
                for path in paths:
                    self.validate(path, ingress=resuming)
            self.robot_pose(route.header.frame_id)
            if not self.navigator.server_is_ready() or not self.follower.server_is_ready():
                raise ValueError('NavigateToPose or FollowPath server is unavailable')
        except (ValueError, TransformException) as exc:
            self.report('blocked', str(exc))
            return
        self.pause_explore.publish(Bool(data=False))
        self.route = route if len(route.poses) > 5000 else deepcopy(route)
        self.work_sections = deepcopy(work_sections)
        self.section_lengths = []
        self.sections = [(start, end, 1) for start, end in
                 (work_sections if work_sections is not None else [(0, len(route.poses) - 1)])]
        self.section_index = 0
        self.execution_path = None
        self.cursor = 0
        self.busy = True
        self.recovery_attempts = 0
        self.stop_state = None
        self.cancel_sent = False
        self.started = time.monotonic()
        self.last_ros_time = self.node.get_clock().now().nanoseconds
        if len(route.poses) > 5000:
            self.phase = 'validating_start'
            self.join_future = self.join_pool.submit(self.validate_initial_route, self.route, resuming)
            self.report('executing', 'Validating work sections before ordinary navigation.', phase=self.phase)
        else:
            self.navigate_to_start()

    def validate_initial_route(self, route, resuming):
        for path in work_paths(route, self.work_sections):
            self.validate(path, ingress=resuming)
        return route

    def navigate_to_start(self):
        self.phase = 'navigating_ingress'
        self.started = time.monotonic()
        goal = NavigateToPose.Goal()
        sections = getattr(self, 'sections', [])
        start = sections[self.section_index][0] if sections else 0
        goal.pose = deepcopy(self.route.poses[start])
        goal.pose.header.frame_id = self.route.header.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        self.report('executing', 'Navigating to the next work start with ordinary navigation.',
                phase=self.phase, stage='transit', work_section=getattr(self, 'section_index', 0))
        self.send(self.navigator, goal)

    def send(self, client, goal):
        self.request_id += 1
        request_id = self.request_id
        try:
            if client is getattr(self, 'navigator', None):
                goal_uuid = UUID(uuid=list(uuid4().bytes))
                self.ingress_goal_id = bytes(goal_uuid.uuid)
                future = client.send_goal_async(goal, goal_uuid=goal_uuid)
            else:
                future = client.send_goal_async(goal)
            future.add_done_callback(lambda result: self.accepted(result, request_id))
        except Exception as exc:
            self.finish('failed', f'Action submission failed: {exc}')

    def accepted(self, future, request_id):
        try:
            handle = future.result()
        except Exception as exc:
            if request_id == self.request_id:
                self.finish('failed', f'Action submission failed: {exc}')
            return
        if request_id != self.request_id:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            state, reason = self.stop_state or ('failed', 'Navigation action was rejected.')
            self.finish(state, reason)
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(lambda result: self.result(result, request_id))
        if self.stop_state:
            self.request_cancel()

    def result(self, future, request_id):
        if request_id != self.request_id:
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self.cancel('blocked', f'Navigation result unavailable; stop is not confirmed: {exc}')
            return
        self.handle = None
        self.cancel_sent = False
        if self.stop_state:
            self.finish(*self.stop_state)
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            if (self.phase == 'following' and wrapped.status == GoalStatus.STATUS_ABORTED
                    and getattr(self, 'recovery_enabled', False)):
                self.begin_recovery()
                return
            self.finish('blocked', f'Navigation stopped with action status {wrapped.status}.')
            return
        if self.phase == 'backing_up':
            self.phase = 'validating_recovery'
            self.started = time.monotonic()
            self.join_future = self.join_pool.submit(self.validate_recovery_route)
            self.report('executing', 'Backup completed; validating the retained route before replanning.',
                        phase=self.phase, recovery_attempt=self.recovery_attempts)
            return
        if self.phase == 'navigating_ingress':
            self.phase = 'waiting_ingress_stop'
            self.started = time.monotonic()
            self.stopped_since = None
            self.report('executing', 'Approach completed; waiting for confirmed stop before coverage.',
                        phase=self.phase, stage='ingress')
        else:
            try:
                self.ready()
                self.check_horizon()
            except (ValueError, TransformException) as exc:
                self.finish('blocked', str(exc))
                return
            if self.tracker.length - self.tracker.progress > 0.12:
                self.finish('blocked', 'Controller reported success before ordered work progress reached the end.')
            else:
                sections = getattr(self, 'sections', [])
                if sections and self.section_index + 1 < len(sections):
                    self.cursor = sections[self.section_index + 1][0]
                    self.wait_for_transfer(self.section_index + 1)
                else:
                    self.finish('completed', 'All coverage work sections completed.')

    def begin_recovery(self):
        try:
            self.ready()
            if self.recovery_attempts >= self.recovery_max_attempts:
                raise ValueError('Coverage recovery attempt limit reached')
            self.recovery_route = self.remaining_path()
            self.recovery_sections = self.remaining_work_sections()
            if self.recovery_route is None or len(self.recovery_route.poses) < 2:
                raise ValueError('No retained route is available for recovery')
            if not getattr(self, 'recovery_backup_enabled', False):
                self.check_current_footprint()
                self.recovery_attempts += 1
                self.phase = 'validating_recovery'
                self.started = time.monotonic()
                self.join_future = self.join_pool.submit(self.validate_recovery_route)
                self.report('executing', 'Controller stopped; validating a planned recovery without blind backup.',
                            phase=self.phase, recovery_attempt=self.recovery_attempts,
                            retained_path_points=len(self.recovery_route.poses))
                return
            if not self.backup.server_is_ready():
                raise ValueError('Nav2 BackUp recovery server is unavailable')
            self.backup_start = self.robot_pose(self.local.frame)
            self.check_backup()
            self.recovery_attempts += 1
            self.phase = 'backing_up'
            self.last_motion_direction = -1
            self.started = time.monotonic()
            goal = BackUp.Goal()
            goal.target.x = self.recovery_backup_distance
            goal.speed = self.recovery_backup_speed
            goal.time_allowance.sec = 8
            self.report('executing', 'Controller stopped; performing a collision-checked short backup.',
                        phase=self.phase, recovery_attempt=self.recovery_attempts,
                        retained_path_points=len(self.recovery_route.poses))
            self.send(self.backup, goal)
        except (ValueError, TransformException) as exc:
            self.finish('blocked', f'Coverage recovery refused: {exc}')

    def check_backup(self):
        current = self.robot_pose(self.local.frame)
        start_x, start_y, start_yaw = pose_xy_yaw(self.backup_start.pose)
        target = deepcopy(self.backup_start)
        target.pose.position.x = start_x - self.recovery_backup_distance * math.cos(start_yaw)
        target.pose.position.y = start_y - self.recovery_backup_distance * math.sin(start_yaw)
        remaining = Path(header=deepcopy(current.header), poses=[current, target])
        self.local.check_path(remaining)

    def validate_recovery_route(self):
        route = self.recovery_route
        for path in work_paths(route, getattr(self, 'recovery_sections', None)):
            self.validate(path, ingress=True)
        return route

    def replan_recovery(self, route):
        self.ready()
        self.robot_pose(route.header.frame_id)
        if not self.navigator.server_is_ready() or not self.follower.server_is_ready():
            raise ValueError('NavigateToPose or FollowPath server is unavailable during recovery')
        self.route = route
        self.work_sections = getattr(self, 'recovery_sections', None)
        self.section_lengths = []
        self.sections = [(start, end, 1) for start, end in
                 (self.work_sections if self.work_sections is not None else [(0, len(route.poses) - 1)])]
        self.section_index = 0
        self.execution_path = None
        self.cursor = 0
        self.navigate_to_start()

    def validate_start_route(self, route):
        sections = getattr(self, 'work_sections', None)
        for path in work_paths(route, sections):
            self.validate(path, ingress=True)
        return route

    def check_ingress_stop(self):
        if time.monotonic() - self.started > 5.0:
            raise ValueError('Coverage start stop timed out')
        if not self.stop_confirmed():
            return
        position_x, position_y, yaw = pose_xy_yaw(self.robot_pose(self.route.header.frame_id).pose)
        if not all(math.isfinite(value) for value in (position_x, position_y, yaw)):
            raise ValueError('Coverage start pose must be finite')
        sections = getattr(self, 'sections', [])
        start = sections[self.section_index][0] if sections else 0
        target_x, target_y, target_yaw = pose_xy_yaw(self.route.poses[start].pose)
        self.report('executing', 'Approach stopped; validating coverage from the actual pose.',
                    phase=self.phase, start_position_error_m=math.hypot(position_x - target_x, position_y - target_y),
                    start_heading_error_rad=abs(math.atan2(math.sin(yaw - target_yaw), math.cos(yaw - target_yaw))))
        if len(self.route.poses) > 5000:
            self.phase = 'validating_ingress'
            self.started = time.monotonic()
            self.join_future = self.join_pool.submit(self.validate_start_route, self.route)
        else:
            self.begin_following(self.validate_start_route(self.route))

    def begin_following(self, path):
        self.ready()
        self.execution_path = path
        if not getattr(self, 'sections', []):
            self.sections = [(0, len(path.poses) - 1, 1)]
            self.section_index = 0
        self.send_motion_section()

    def send_motion_section(self):
        self.ready()
        start, end, _ = self.sections[self.section_index]
        path = Path(header=deepcopy(self.execution_path.header),
                    poses=deepcopy(self.execution_path.poses[start:end + 1]))
        path.header.stamp = self.node.get_clock().now().to_msg()
        for stamped in path.poses:
            stamped.header.stamp = path.header.stamp
        self.tracker = OrderedPathTracker([(pose.pose.position.x, pose.pose.position.y)
                                          for pose in path.poses], search_distance=0.6)
        self.section_tracker = self.tracker
        if not getattr(self, 'section_lengths', []):
            self.section_lengths = [sum(
                math.hypot(second.pose.position.x - first.pose.position.x,
                           second.pose.position.y - first.pose.position.y)
                for first, second in zip(self.execution_path.poses[begin:finish],
                                         self.execution_path.poses[begin + 1:finish + 1]))
                for begin, finish, _ in self.sections]
        self.section_offset = sum(self.section_lengths[:self.section_index])
        self.total_work_length = sum(self.section_lengths)
        self.phase = 'following'
        self.plan_pub.publish(path)
        self.check_horizon()
        control_publisher = getattr(self, 'control_plan_pub', None)
        if control_publisher is not None:
            control_publisher.publish(path)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'CoverageFollowPath'
        goal.goal_checker_id = 'coverage_goal_checker'
        self.report('executing', 'Following the validated work section.', phase=self.phase,
                    path_points=len(path.poses), path_length_m=self.tracker.length,
                    motion_direction='forward',
                    motion_section=self.section_index)
        self.send(self.follower, goal)

    def wait_for_transfer(self, section_index):
        self.pending_section_index = section_index
        self.phase = 'waiting_work_stop'
        self.started = time.monotonic()
        self.stopped_since = None
        self.report('executing', 'Waiting for confirmed stop before ordinary transit.', phase=self.phase,
                    path_index=self.cursor)

    def check_current_footprint(self):
        current = self.robot_pose(self.local.frame)
        self.local.check_path(Path(header=current.header, poses=[current]))

    def check_work_stop(self):
        if time.monotonic() - self.started > 5.0:
            raise ValueError('Work-section stop timed out')
        if self.stop_confirmed():
            self.section_index = self.pending_section_index
            self.pending_section_index = None
            self.navigate_to_start()

    def stop_confirmed(self):
        self.check_current_footprint()
        age = (self.node.get_clock().now().nanoseconds - self.motion_stamp.nanoseconds) / 1e9
        if (not -0.1 <= age <= 0.5 or time.monotonic() - self.motion_received > 0.5
                or not math.isfinite(self.motion_linear) or not math.isfinite(self.motion_angular)):
            raise ValueError('Fresh finite odometry is required to confirm a stop')
        if self.motion_linear > 0.02 or self.motion_angular > 0.05:
            self.stopped_since = None
            return False
        if self.stopped_since is None:
            self.stopped_since = time.monotonic()
            return False
        return (time.monotonic() - self.stopped_since >= 0.3
                and self.motion_received - self.stopped_since >= 0.3)

    def check_horizon(self):
        pose = self.robot_pose(self.execution_path.header.frame_id)
        position_x, position_y, yaw = pose_xy_yaw(pose.pose)
        tracker = getattr(self, 'section_tracker', None) or self.tracker
        tracking = tracker.update(position_x, position_y, yaw)
        if tracking is None:
            raise ValueError('Ordered path tracking is unavailable')
        tracking_warning = not tracking['tracking_valid'] or tracking['path_distance_m'] > 0.15
        if tracking_warning and time.monotonic() - getattr(self, 'last_tracking_warning', 0.0) >= 2.0:
            self.last_tracking_warning = time.monotonic()
            self.node.get_logger().warn(
                f"Coverage tracking deviation: {tracking['path_distance_m']:.3f} m; "
                'MPPI collision checking remains active')
        if tracking['tracking_valid']:
            sections = getattr(self, 'sections', [])
            start = sections[self.section_index][0] if sections else 0
            self.cursor = max(self.cursor, start + tracking['segment'])
        self.check_current_footprint()
        if time.monotonic() - self.last_feedback >= 1.0:
            self.last_feedback = time.monotonic()
            self.report('executing', 'Continuous coverage in progress.', phase=self.phase,
                        stage='coverage',
                        tracking_warning=tracking_warning, path_distance_m=tracking['path_distance_m'],
                        path_index=self.cursor, progress_m=getattr(self, 'section_offset', 0.0) + self.tracker.progress,
                        distance_remaining=max(0.0, getattr(self, 'total_work_length', self.tracker.length)
                            - getattr(self, 'section_offset', 0.0) - self.tracker.progress))

    def guard(self):
        if not self.busy:
            return
        if self.stop_state:
            if self.handle and not self.cancel_sent:
                self.request_cancel()
            elif self.phase in ('waiting_work_stop', 'waiting_ingress_stop'):
                self.finish(*self.stop_state)
            elif (self.phase in ('validating_start', 'validating_ingress', 'validating_recovery')
                  and (self.join_future is None or self.join_future.done())):
                self.join_future = None
                self.finish(*self.stop_state)
            return
        try:
            now = self.node.get_clock().now().nanoseconds
            if self.last_ros_time is not None and now < self.last_ros_time:
                self.route = self.execution_path = None
                raise ValueError('Simulation clock reset; preview a new route')
            self.last_ros_time = now
            self.ready()
            if self.phase == 'navigating_ingress':
                self.check_current_footprint()
            if self.phase in ('validating_start', 'validating_ingress', 'validating_recovery'):
                if self.join_future.done():
                    future, self.join_future = self.join_future, None
                    try:
                        if self.phase == 'validating_start':
                            future.result()
                            self.robot_pose(self.route.header.frame_id)
                            self.navigate_to_start()
                        elif self.phase == 'validating_recovery':
                            self.replan_recovery(future.result())
                        else:
                            self.begin_following(future.result())
                    except (ValueError, TransformException) as exc:
                        self.finish('blocked', str(exc))
                        return
                elif time.monotonic() - self.started > 120.0:
                    raise ValueError('Ingress validation timed out')
            if self.phase == 'backing_up':
                if time.monotonic() - self.started > 15.0:
                    raise ValueError('Coverage backup recovery timed out')
                self.check_backup()
            if self.phase == 'waiting_work_stop':
                self.check_work_stop()
            if self.phase == 'waiting_ingress_stop':
                self.check_ingress_stop()
            if self.phase == 'following':
                self.check_horizon()
        except (ValueError, TransformException) as exc:
            self.cancel('blocked', str(exc))

    def cancel(self, state='canceled', reason='Coverage canceled by operator.'):
        if not self.busy:
            return
        if self.stop_state is None:
            self.stop_state = (state, reason)
            self.report('cancel_requested', reason)
        if self.handle and not self.cancel_sent:
            self.request_cancel()

    def request_cancel(self):
        self.cancel_sent = True
        self.handle.cancel_goal_async().add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.cancel_sent = False
        except Exception:
            self.cancel_sent = False

    def finish(self, state, reason):
        self.busy = False
        self.phase = state
        self.handle = None
        self.plan_pub.publish(Path())
        control_publisher = getattr(self, 'control_plan_pub', None)
        if control_publisher is not None:
            control_publisher.publish(Path())
        self.report(state, reason, path_index=self.cursor)

    def forget(self):
        if self.busy:
            raise ValueError('Cannot discard an active execution')
        self.route = self.execution_path = self.tracker = None
        self.sections = []
        self.work_sections = None
        self.section_lengths = []
        self.section_tracker = None
        self.phase = 'idle'
        self.plan_pub.publish(Path())

    def close(self):
        self.join_pool.shutdown(wait=False, cancel_futures=True)

    def remaining_path(self):
        path = self.execution_path if self.execution_path is not None else self.route
        if path is None or self.cursor == 0:
            return path
        return Path(header=deepcopy(path.header), poses=path.poses[self.cursor:])

    def remaining_work_sections(self):
        sections = getattr(self, 'work_sections', None)
        if sections is None:
            return None
        return [[max(start, self.cursor) - self.cursor, end - self.cursor]
                for start, end in sections if end >= self.cursor]