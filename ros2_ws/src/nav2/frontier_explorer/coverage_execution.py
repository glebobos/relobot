"""Single-action coverage execution with validated ingress and bounded stop guards."""

from copy import deepcopy
import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_explorer.coverage_path import (
    FreeSpaceValidator, join_forward_paths, pose_xy_yaw,
)
from frontier_explorer.navigation_metrics import OrderedPathTracker


class CoverageExecution:
    def __init__(self, node, report, validate, footprint, clearance, min_radius):
        self.node, self.report, self.validate = node, report, validate
        self.footprint, self.clearance, self.min_radius = footprint, clearance, min_radius
        self.planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        self.follower = ActionClient(node, FollowPath, 'follow_path')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.plan_pub = node.create_publisher(Path, '/coverage/execution_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pause_explore = node.create_publisher(Bool, '/explore/resume', 1)
        self.busy = False
        self.handle = None
        self.phase = 'idle'
        self.stop_state = None
        self.route = None
        self.execution_path = None
        self.tracker = None
        self.cursor = 0
        self.last_feedback = 0.0
        self.local = None
        self.local_received = 0.0
        self.scan_received = 0.0
        self.scan_stamp = Time()
        self.last_ros_time = None
        self.other_navigation = {}
        self.started = 0.0
        self.cancel_sent = False
        self.request_id = 0
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_costmap,
                                 qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for action in ('navigate_to_pose', 'navigate_through_poses'):
            node.create_subscription(GoalStatusArray, f'/{action}/_action/status',
                                     lambda msg, name=action: self.on_navigation(name, msg), qos)
        self.timer = node.create_timer(0.2, self.guard, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_navigation(self, name, message):
        self.other_navigation[name] = any(
            item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                            GoalStatus.STATUS_CANCELING) for item in message.status_list)
        if self.busy and self.other_navigation[name]:
            self.cancel('blocked', 'Another navigation action became active; coverage interrupted.')

    def on_scan(self, message):
        self.scan_received = time.monotonic()
        self.scan_stamp = Time.from_msg(message.header.stamp)

    def on_costmap(self, message):
        try:
            self.local = FreeSpaceValidator(message, self.footprint, self.clearance,
                                            occupied_threshold=100)
            self.local_stamp = Time.from_msg(message.header.stamp)
            self.local_received = time.monotonic()
        except ValueError:
            self.local = None

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
            raise ValueError('Laser scan timestamp is stale')
        age = (self.node.get_clock().now().nanoseconds - self.local_stamp.nanoseconds) / 1e9
        if not -0.1 <= age <= 1.0:
            raise ValueError('Local costmap timestamp is stale')
        if any(self.other_navigation.values()):
            raise ValueError('Stop exploration and other navigation before starting coverage')

    def start(self, route, resuming=False):
        if self.busy:
            self.report('busy', 'Coverage is still handling an action.')
            return
        try:
            self.ready()
            self.validate(route, ingress=resuming)
            start = self.robot_pose(route.header.frame_id)
            if not self.planner.server_is_ready() or not self.follower.server_is_ready():
                raise ValueError('Planner or FollowPath server is unavailable')
        except (ValueError, TransformException) as exc:
            self.report('blocked', str(exc))
            return
        self.pause_explore.publish(Bool(data=False))
        self.route = deepcopy(route)
        self.execution_path = None
        self.cursor = 0
        self.busy = True
        self.stop_state = None
        self.cancel_sent = False
        self.started = time.monotonic()
        self.last_ros_time = self.node.get_clock().now().nanoseconds
        self.phase = 'planning_ingress'
        self.report('executing', 'Planning a forward-only ingress to the coverage route.', phase=self.phase)
        goal = ComputePathToPose.Goal()
        goal.start = start
        goal.use_start = True
        goal.goal = deepcopy(route.poses[0])
        goal.goal.header = start.header
        goal.planner_id = 'CoverageIngress'
        self.send(self.planner, goal)

    def send(self, client, goal):
        self.request_id += 1
        request_id = self.request_id
        try:
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
            self.finish('blocked', f'Navigation stopped with action status {wrapped.status}.')
            return
        if self.phase == 'planning_ingress':
            try:
                ingress = wrapped.result.path
                path = join_forward_paths(ingress, self.route, self.min_radius,
                                          lambda candidate: self.validate(candidate, ingress=True))
                self.ready()
                self.execution_path = path
                self.tracker = OrderedPathTracker([(pose.pose.position.x, pose.pose.position.y)
                                                  for pose in path.poses], search_distance=0.6)
                self.phase = 'following'
                self.check_horizon()
                self.plan_pub.publish(path)
                goal = FollowPath.Goal()
                goal.path = path
                goal.controller_id = 'CoverageFollowPath'
                goal.goal_checker_id = 'coverage_goal_checker'
                self.report('executing', 'Following the validated continuous route.', phase=self.phase,
                            path_points=len(path.poses), path_length_m=self.tracker.length)
                self.send(self.follower, goal)
            except (ValueError, TransformException) as exc:
                self.finish('blocked', str(exc))
        else:
            if self.tracker.length - self.tracker.progress > 0.12:
                self.finish('blocked', 'Controller reported success before ordered route progress reached the end.')
            else:
                self.finish('completed', 'Continuous coverage route completed.')

    def check_horizon(self):
        pose = self.robot_pose(self.execution_path.header.frame_id)
        position_x, position_y, yaw = pose_xy_yaw(pose.pose)
        tracking = self.tracker.update(position_x, position_y, yaw)
        if not tracking or not tracking['tracking_valid'] or tracking['path_distance_m'] > 0.15:
            raise ValueError('Robot left the ordered coverage corridor')
        self.cursor = max(self.cursor, tracking['segment'])
        horizon = Path()
        horizon.header = self.execution_path.header
        horizon.poses = [pose]
        distance = 0.0
        previous = pose
        for target in self.execution_path.poses[self.cursor + 1:]:
            distance += math.hypot(target.pose.position.x - previous.pose.position.x,
                                   target.pose.position.y - previous.pose.position.y)
            horizon.poses.append(target)
            previous = target
            if distance >= 0.8:
                break
        transform = self.tf.lookup_transform(self.local.frame, horizon.header.frame_id,
                                              self.local_stamp).transform
        transform_pose = PoseStamped().pose
        transform_pose.position.x = transform.translation.x
        transform_pose.position.y = transform.translation.y
        transform_pose.orientation = transform.rotation
        self.local.check_path(horizon, pose_xy_yaw(transform_pose))
        if time.monotonic() - self.last_feedback >= 1.0:
            self.last_feedback = time.monotonic()
            self.report('executing', 'Continuous coverage in progress.', phase=self.phase,
                        path_index=self.cursor, progress_m=self.tracker.progress,
                        distance_remaining=max(0.0, self.tracker.length - self.tracker.progress))

    def guard(self):
        if not self.busy:
            return
        if self.stop_state:
            if self.handle and not self.cancel_sent:
                self.request_cancel()
            return
        try:
            now = self.node.get_clock().now().nanoseconds
            if self.last_ros_time is not None and now < self.last_ros_time:
                self.route = self.execution_path = None
                raise ValueError('Simulation clock reset; preview a new route')
            self.last_ros_time = now
            self.ready()
            if self.phase == 'planning_ingress' and time.monotonic() - self.started > 10.0:
                raise ValueError('Ingress planning timed out')
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
        self.report(state, reason, path_index=self.cursor)

    def forget(self):
        if self.busy:
            raise ValueError('Cannot discard an active execution')
        self.route = self.execution_path = self.tracker = None
        self.phase = 'idle'
        self.plan_pub.publish(Path())

    def remaining_path(self):
        if self.execution_path is None:
            return deepcopy(self.route)
        path = deepcopy(self.execution_path)
        path.poses = path.poses[self.cursor:]
        return path