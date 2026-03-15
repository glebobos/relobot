from __future__ import annotations

import copy
import json
import math
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PolygonStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class CoverageManager(Node):
    def __init__(self) -> None:
        super().__init__('coverage_manager')

        self.declare_parameter('polygon_topic', '/coverage/polygon')
        self.declare_parameter('polygon_echo_topic', '/coverage/polygon_active')
        self.declare_parameter('command_topic', '/coverage/command')
        self.declare_parameter('status_topic', '/coverage/status')
        self.declare_parameter('preview_path_topic', '/coverage/preview_path')
        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('compute_coverage_action_name', 'compute_coverage_path')
        self.declare_parameter('compute_path_to_pose_action_name', 'compute_path_to_pose')
        self.declare_parameter('follow_path_action_name', 'follow_path')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        self.declare_parameter('headland_width', 0.0)
        self.declare_parameter('path_continuity_type', 'CONTINUOUS')
        self.declare_parameter('path_type', 'DUBIN')
        self.declare_parameter('turn_point_distance', 0.1)
        self.declare_parameter('action_wait_timeout_sec', 5.0)
        self.declare_parameter('allow_execute_without_fresh_preview', True)

        self._default_frame_id = self.get_parameter('default_frame_id').value
        self._planner_id = self.get_parameter('planner_id').value
        self._controller_id = self.get_parameter('controller_id').value
        self._goal_checker_id = self.get_parameter('goal_checker_id').value
        self._headland_width = float(self.get_parameter('headland_width').value)
        self._path_continuity_type = self.get_parameter('path_continuity_type').value
        self._path_type = self.get_parameter('path_type').value
        self._turn_point_distance = float(self.get_parameter('turn_point_distance').value)
        self._action_wait_timeout_sec = float(self.get_parameter('action_wait_timeout_sec').value)
        self._allow_execute_without_fresh_preview = bool(
            self.get_parameter('allow_execute_without_fresh_preview').value
        )

        compute_action_name = self.get_parameter('compute_coverage_action_name').value
        compute_path_to_pose_action_name = self.get_parameter(
            'compute_path_to_pose_action_name'
        ).value
        follow_path_action_name = self.get_parameter('follow_path_action_name').value

        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10
        )
        self._preview_path_pub = self.create_publisher(
            Path, self.get_parameter('preview_path_topic').value, 10
        )
        self._polygon_echo_pub = self.create_publisher(
            PolygonStamped, self.get_parameter('polygon_echo_topic').value, 10
        )

        self.create_subscription(
            PolygonStamped,
            self.get_parameter('polygon_topic').value,
            self._on_polygon,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('command_topic').value,
            self._on_command,
            10,
        )

        self._compute_client = ActionClient(self, ComputeCoveragePath, compute_action_name)
        self._planner_client = ActionClient(
            self, ComputePathToPose, compute_path_to_pose_action_name
        )
        self._follow_client = ActionClient(self, FollowPath, follow_path_action_name)

        self._polygon_msg = PolygonStamped()
        self._cached_path = Path()
        self._compute_goal_handle = None
        self._planner_goal_handle = None
        self._follow_goal_handle = None
        self._state = 'idle'
        self._last_preview_valid = False

        self._publish_status('idle', 'Coverage manager ready.')

    def _on_polygon(self, msg: PolygonStamped) -> None:
        normalized = self._normalize_polygon(msg)
        if normalized is None:
            self._polygon_msg = PolygonStamped()
            self._publish_status('polygon_invalid', 'Polygon needs at least 3 unique points.')
            return

        self._polygon_msg = normalized
        self._polygon_echo_pub.publish(self._polygon_msg)
        self._last_preview_valid = False
        self._publish_status(
            'polygon_ready',
            'Coverage polygon updated.',
            point_count=len(self._polygon_msg.polygon.points) - 1,
            frame_id=self._polygon_msg.header.frame_id,
        )

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == 'preview':
            self._start_preview()
            return
        if command == 'execute':
            self._start_execution()
            return
        if command in {'cancel', 'stop'}:
            self._cancel_active_goals()
            return
        if command == 'clear':
            self._clear_cached_state()
            return

        self._publish_status('error', f'Unknown coverage command: {command}')

    def _start_preview(self) -> None:
        if self._compute_goal_handle or self._planner_goal_handle or self._follow_goal_handle:
            self._publish_status('busy', 'Coverage manager is already handling a request.')
            return
        if len(self._polygon_msg.polygon.points) < 4:
            self._publish_status('polygon_invalid', 'Draw a closed polygon before previewing.')
            return
        if not self._compute_client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._publish_status('server_unavailable', 'Coverage server action is not available.')
            return

        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = True
        goal.generate_route = True
        goal.generate_path = True
        goal.frame_id = self._polygon_msg.header.frame_id or self._default_frame_id

        polygon = Coordinates()
        for point in self._polygon_msg.polygon.points:
            coordinate = Coordinate()
            coordinate.axis1 = float(point.x)
            coordinate.axis2 = float(point.y)
            polygon.coordinates.append(coordinate)
        goal.polygons.append(polygon)

        goal.headland_mode.mode = 'CONSTANT'
        goal.headland_mode.width = float(self._headland_width)
        goal.path_mode.mode = self._path_type
        goal.path_mode.continuity_mode = self._path_continuity_type
        goal.path_mode.turn_point_distance = float(self._turn_point_distance)

        self._state = 'planning'
        self._publish_status(
            'planning',
            'Computing coverage path.',
            frame_id=goal.frame_id,
            point_count=len(self._polygon_msg.polygon.points) - 1,
        )

        send_future = self._compute_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_preview_goal_response)

    def _on_preview_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._compute_goal_handle = None
            self._publish_status('error', f'Coverage preview request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._compute_goal_handle = None
            self._publish_status('rejected', 'Coverage preview request was rejected.')
            return

        self._compute_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_preview_result)

    def _on_preview_result(self, future) -> None:
        self._compute_goal_handle = None
        try:
            wrapped_result = future.result()
        except Exception as exc:  # pragma: no cover
            self._publish_status('error', f'Coverage preview result failed: {exc}')
            return

        status = wrapped_result.status
        result = wrapped_result.result
        if status == GoalStatus.STATUS_CANCELED:
            self._publish_status('canceled', 'Coverage preview canceled.')
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_status('failed', f'Coverage preview failed with status {status}.')
            return
        if result.error_code != ComputeCoveragePath.Result.NONE:
            self._publish_status('failed', f'Coverage server returned error code {result.error_code}.')
            return
        if not result.nav_path.poses:
            self._publish_status('failed', 'Coverage server returned an empty path.')
            return

        self._cached_path = copy.deepcopy(result.nav_path)
        self._stamp_path(self._cached_path)
        self._preview_path_pub.publish(self._cached_path)
        self._last_preview_valid = True
        self._state = 'preview_ready'
        self._publish_status(
            'preview_ready',
            'Coverage path ready for inspection or execution.',
            pose_count=len(self._cached_path.poses),
        )

    def _start_execution(self) -> None:
        if self._compute_goal_handle or self._planner_goal_handle or self._follow_goal_handle:
            self._publish_status('busy', 'Coverage manager is already handling a request.')
            return
        if not self._cached_path.poses:
            self._publish_status('no_preview', 'Preview a coverage path before execution.')
            return
        if not self._last_preview_valid and not self._allow_execute_without_fresh_preview:
            self._publish_status('stale_preview', 'Preview the current polygon again before execution.')
            return
        if not self._planner_client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._publish_status('server_unavailable', 'ComputePathToPose action is not available.')
            return
        if not self._follow_client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._publish_status('server_unavailable', 'FollowPath action is not available.')
            return

        goal = ComputePathToPose.Goal()
        goal.goal = copy.deepcopy(self._cached_path.poses[0])
        goal.goal.header.frame_id = goal.goal.header.frame_id or self._default_frame_id
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.planner_id = self._planner_id
        goal.use_start = False

        self._state = 'planning'
        self._publish_status(
            'planning',
            'Planning route to the coverage start.',
            pose_count=len(self._cached_path.poses),
        )

        send_future = self._planner_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_approach_goal_response)

    def _on_approach_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._planner_goal_handle = None
            self._publish_status('error', f'Approach planning request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._planner_goal_handle = None
            self._publish_status('rejected', 'Approach planning request was rejected.')
            return

        self._planner_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_approach_result)

    def _on_approach_result(self, future) -> None:
        self._planner_goal_handle = None
        try:
            wrapped_result = future.result()
        except Exception as exc:  # pragma: no cover
            self._publish_status('error', f'Approach planning result failed: {exc}')
            return

        status = wrapped_result.status
        result = wrapped_result.result
        if status == GoalStatus.STATUS_CANCELED:
            self._publish_status('canceled', 'Approach planning canceled.')
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_status('failed', f'Approach planning failed with status {status}.')
            return
        if not result.path.poses:
            self._publish_status('failed', 'Planner returned an empty route to the coverage start.')
            return

        execution_path = self._merge_paths(result.path, self._cached_path)
        self._send_follow_path(execution_path)

    def _send_follow_path(self, path: Path) -> None:
        goal = FollowPath.Goal()
        goal.path = copy.deepcopy(path)
        self._stamp_path(goal.path)
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id

        self._state = 'executing'
        self._publish_status(
            'executing',
            'Executing route to coverage start and coverage path.',
            pose_count=len(goal.path.poses),
        )

        send_future = self._follow_client.send_goal_async(goal, feedback_callback=self._on_follow_feedback)
        send_future.add_done_callback(self._on_execute_goal_response)

    def _on_execute_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._follow_goal_handle = None
            self._publish_status('error', f'Coverage execution request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._follow_goal_handle = None
            self._publish_status('rejected', 'Coverage execution request was rejected.')
            return

        self._follow_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_execute_result)

    def _on_execute_result(self, future) -> None:
        self._follow_goal_handle = None
        try:
            wrapped_result = future.result()
        except Exception as exc:  # pragma: no cover
            self._publish_status('error', f'Coverage execution result failed: {exc}')
            return

        status = wrapped_result.status
        if status == GoalStatus.STATUS_CANCELED:
            self._publish_status('canceled', 'Coverage execution canceled.')
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_status('failed', f'Coverage execution failed with status {status}.')
            return

        self._state = 'completed'
        self._publish_status('completed', 'Coverage execution completed successfully.')

    def _on_follow_feedback(self, _goal_handle, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._publish_status(
            'executing',
            'Coverage execution in progress.',
            distance_to_goal=round(float(feedback.distance_to_goal), 3),
            speed=round(float(feedback.speed), 3),
        )

    def _cancel_active_goals(self) -> None:
        canceled = False
        if self._compute_goal_handle is not None:
            canceled = True
            self._compute_goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_done)
        if self._planner_goal_handle is not None:
            canceled = True
            self._planner_goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_done)
        if self._follow_goal_handle is not None:
            canceled = True
            self._follow_goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_done)

        if canceled:
            self._state = 'cancel_requested'
            self._publish_status('cancel_requested', 'Canceling active coverage task.')
        else:
            self._publish_status('idle', 'No active coverage task to cancel.')

    def _on_cancel_done(self, _future) -> None:
        self._publish_status('canceled', 'Coverage task canceled.')

    def _clear_cached_state(self) -> None:
        self._cancel_active_goals()
        self._polygon_msg = PolygonStamped()
        self._cached_path = Path()
        self._last_preview_valid = False
        self._preview_path_pub.publish(Path())
        self._polygon_echo_pub.publish(PolygonStamped())
        self._state = 'idle'
        self._publish_status('idle', 'Coverage polygon and cached path cleared.')

    def _normalize_polygon(self, msg: PolygonStamped) -> PolygonStamped | None:
        points = []
        for point in msg.polygon.points:
            candidate = (float(point.x), float(point.y))
            if not points or not self._points_equal(points[-1], candidate):
                points.append(candidate)

        if len(points) >= 2 and self._points_equal(points[0], points[-1]):
            points.pop()

        if len(points) < 3:
            return None

        normalized = PolygonStamped()
        normalized.header = msg.header
        normalized.header.frame_id = msg.header.frame_id or self._default_frame_id

        for x_coord, y_coord in points:
            normalized.polygon.points.append(Point32(x=float(x_coord), y=float(y_coord), z=0.0))

        first = normalized.polygon.points[0]
        normalized.polygon.points.append(Point32(x=float(first.x), y=float(first.y), z=0.0))
        return normalized

    def _points_equal(self, first: tuple[float, float], second: tuple[float, float]) -> bool:
        return math.isclose(first[0], second[0], abs_tol=1e-4) and math.isclose(
            first[1], second[1], abs_tol=1e-4
        )

    def _stamp_path(self, path: Path) -> None:
        stamp = self.get_clock().now().to_msg()
        frame_id = path.header.frame_id or self._polygon_msg.header.frame_id or self._default_frame_id
        path.header.stamp = stamp
        path.header.frame_id = frame_id
        for pose in path.poses:
            pose.header.stamp = stamp
            pose.header.frame_id = frame_id

    def _merge_paths(self, approach_path: Path, coverage_path: Path) -> Path:
        merged = Path()
        merged.header.frame_id = (
            coverage_path.header.frame_id or approach_path.header.frame_id or self._default_frame_id
        )
        merged.poses = [copy.deepcopy(pose) for pose in approach_path.poses]

        coverage_poses = [copy.deepcopy(pose) for pose in coverage_path.poses]
        if merged.poses and coverage_poses and self._same_pose(merged.poses[-1], coverage_poses[0]):
            coverage_poses = coverage_poses[1:]

        merged.poses.extend(coverage_poses)
        return merged

    def _same_pose(self, first_pose, second_pose) -> bool:
        first = first_pose.pose.position
        second = second_pose.pose.position
        return math.isclose(first.x, second.x, abs_tol=1e-3) and math.isclose(
            first.y, second.y, abs_tol=1e-3
        )

    def _publish_status(self, state: str, message: str, **extra: Any) -> None:
        self._state = state
        payload = {'state': state, 'message': message}
        payload.update(extra)
        self._status_pub.publish(String(data=json.dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoverageManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()