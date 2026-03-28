from __future__ import annotations

import json
import math
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Pose, PoseStamped, PolygonStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
import cv2
import numpy as np
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class CoverageManager(Node):
    def __init__(self) -> None:
        super().__init__('coverage_manager')

        self.declare_parameter('polygon_echo_topic', '/coverage/polygon_active')
        self.declare_parameter('command_topic', '/coverage/command')
        self.declare_parameter('status_topic', '/coverage/status')
        self.declare_parameter('preview_path_topic', '/coverage/preview_path')
        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('compute_coverage_action_name', 'compute_coverage_path')
        self.declare_parameter('navigate_through_poses_action_name', 'navigate_through_poses')
        self.declare_parameter('headland_width', 0.5)
        self.declare_parameter('path_continuity_type', 'CONTINUOUS')
        self.declare_parameter('path_type', 'REEDS_SHEPP')
        self.declare_parameter('turn_point_distance', 0.1)
        self.declare_parameter('action_wait_timeout_sec', 5.0)
        self.declare_parameter('allow_execute_without_fresh_preview', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_contour_epsilon', 0.5)
        self.declare_parameter('map_morph_close_radius', 3)
        self.declare_parameter('map_erode_radius', 5)
        self.declare_parameter('swath_endpoint_margin', 0.25)

        self._default_frame_id = self.get_parameter('default_frame_id').value
        self._headland_width = float(self.get_parameter('headland_width').value)
        self._path_continuity_type = self.get_parameter('path_continuity_type').value
        self._path_type = self.get_parameter('path_type').value
        self._turn_point_distance = float(self.get_parameter('turn_point_distance').value)
        self._action_wait_timeout_sec = float(self.get_parameter('action_wait_timeout_sec').value)
        self._allow_execute_without_fresh_preview = bool(
            self.get_parameter('allow_execute_without_fresh_preview').value
        )

        compute_action_name = self.get_parameter('compute_coverage_action_name').value
        nav_through_poses_action_name = self.get_parameter('navigate_through_poses_action_name').value

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
            String,
            self.get_parameter('command_topic').value,
            self._on_command,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._on_map,
            10,
        )

        self._compute_client = ActionClient(self, ComputeCoveragePath, compute_action_name)
        self._nav_client = ActionClient(self, NavigateThroughPoses, nav_through_poses_action_name)

        self._polygon_msg = PolygonStamped()
        self._cached_waypoints: list = []
        self._cached_path = Path()
        self._compute_goal_handle = None
        self._nav_goal_handle = None
        self._state = 'idle'
        self._last_preview_valid = False
        self._last_map_msg = None

        self._publish_status('idle', 'Coverage manager ready.')

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
        if command == 'refresh_map':
            if self._last_map_msg is not None:
                prev_state = self._state
                self._state = 'idle'
                self._on_map(self._last_map_msg)
                if self._state == 'idle':
                    self._state = prev_state
            else:
                self._publish_status('error', 'No map received yet.')
            return

        self._publish_status('error', f'Unknown coverage command: {command}')

    def _start_preview(self) -> None:
        if self._compute_goal_handle or self._nav_goal_handle:
            self._publish_status('busy', 'Coverage manager is already handling a request.')
            return
        if len(self._polygon_msg.polygon.points) < 4:
            self._publish_status('polygon_invalid', 'No map boundary yet. Wait for SLAM map or click Refresh Map.')
            return
        if not self._compute_client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._publish_status('server_unavailable', 'Coverage server action is not available.')
            return

        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = True
        goal.generate_route = True
        goal.generate_path = False
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
        if not result.coverage_path.swaths:
            self._publish_status('failed', 'Coverage server returned no swaths.')
            return

        frame_id = self._polygon_msg.header.frame_id or self._default_frame_id
        self._cached_waypoints = self._swath_to_waypoints(
            result.coverage_path.swaths, frame_id
        )

        # Build a preview path from straight swath lines (no turn arcs)
        preview = Path()
        preview.header.frame_id = frame_id
        preview.header.stamp = self.get_clock().now().to_msg()
        for wp in self._cached_waypoints:
            ps = PoseStamped()
            ps.header = wp.header
            ps.pose = wp.pose
            preview.poses.append(ps)
        self._cached_path = preview
        self._preview_path_pub.publish(preview)
        self._last_preview_valid = True
        self._state = 'preview_ready'
        self._publish_status(
            'preview_ready',
            'Coverage path ready for inspection or execution.',
            swath_count=len(result.coverage_path.swaths),
            waypoint_count=len(self._cached_waypoints),
        )

    def _swath_to_waypoints(self, swaths, frame_id: str) -> list:
        waypoints = []
        endpoint_margin = float(self.get_parameter('swath_endpoint_margin').value)
        for swath in swaths:
            sx, sy = float(swath.start.x), float(swath.start.y)
            ex, ey = float(swath.end.x), float(swath.end.y)
            dx = ex - sx
            dy = ey - sy
            length = math.hypot(dx, dy)
            if length > (2.0 * endpoint_margin) and endpoint_margin > 0.0:
                offset_x = (dx / length) * endpoint_margin
                offset_y = (dy / length) * endpoint_margin
                sx += offset_x
                sy += offset_y
                ex -= offset_x
                ey -= offset_y
            angle = math.atan2(ey - sy, ex - sx)
            qz = math.sin(angle / 2.0)
            qw = math.cos(angle / 2.0)
            for px, py in [(sx, sy), (ex, ey)]:
                ps = PoseStamped()
                ps.header.frame_id = frame_id
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = px
                ps.pose.position.y = py
                ps.pose.position.z = 0.0
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw
                waypoints.append(ps)
        return waypoints

    def _start_execution(self) -> None:
        if self._compute_goal_handle or self._nav_goal_handle:
            self._publish_status('busy', 'Coverage manager is already handling a request.')
            return
        if not self._cached_waypoints:
            self._publish_status('no_preview', 'Preview a coverage path before execution.')
            return
        if not self._last_preview_valid and not self._allow_execute_without_fresh_preview:
            self._publish_status('stale_preview', 'Preview the current polygon again before execution.')
            return
        if not self._nav_client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._publish_status('server_unavailable', 'NavigateThroughPoses action is not available.')
            return

        stamp = self.get_clock().now().to_msg()
        goal = NavigateThroughPoses.Goal()
        for wp in self._cached_waypoints:
            wp.header.stamp = stamp
            goal.poses.append(wp)

        self._state = 'executing'
        self._publish_status(
            'executing',
            'Executing coverage path.',
            waypoint_count=len(goal.poses),
        )

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback
        )
        send_future.add_done_callback(self._on_execute_goal_response)

    def _on_execute_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._nav_goal_handle = None
            self._publish_status('error', f'Coverage execution request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._nav_goal_handle = None
            self._publish_status('rejected', 'Coverage execution request was rejected.')
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_execute_result)

    def _on_execute_result(self, future) -> None:
        self._nav_goal_handle = None
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

    def _on_nav_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._publish_status(
            'executing',
            'Coverage execution in progress.',
            distance_remaining=round(float(feedback.distance_remaining), 2),
        )

    def _cancel_active_goals(self) -> None:
        canceled = False
        if self._compute_goal_handle is not None:
            canceled = True
            self._compute_goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_done)
        if self._nav_goal_handle is not None:
            canceled = True
            self._nav_goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_done)

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
        self._cached_waypoints = []
        self._cached_path = Path()
        self._last_preview_valid = False
        self._preview_path_pub.publish(Path())
        self._polygon_echo_pub.publish(PolygonStamped())
        self._state = 'idle'
        self._publish_status('idle', 'Coverage state cleared.')

    def _on_map(self, msg: OccupancyGrid) -> None:
        h = msg.info.height
        w = msg.info.width
        if h == 0 or w == 0:
            return
        self._last_map_msg = msg
        # Only re-extract polygon when idle — don't overwrite state during
        # planning, preview_ready, executing, or completed.
        if self._state not in ('idle', 'polygon_ready'):
            return
        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        free_mask = np.uint8(np.where(grid == 0, 255, 0))
        # Morphological closing fills small gaps between free cells (SLAM noise)
        # and smooths the boundary so approxPolyDP produces a clean polygon.
        close_r = int(self.get_parameter('map_morph_close_radius').value)
        if close_r > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * close_r + 1, 2 * close_r + 1)
            )
            free_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
        # Erode free space inward so the polygon boundary is kept away from walls.
        # At 0.04 m/cell, 5 px ≈ 0.20 m offset before headland is applied.
        erode_r = int(self.get_parameter('map_erode_radius').value)
        if erode_r > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * erode_r + 1, 2 * erode_r + 1)
            )
            free_mask = cv2.erode(free_mask, kernel)
        contours, _ = cv2.findContours(free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        largest = max(contours, key=cv2.contourArea)
        epsilon_px = self.get_parameter('map_contour_epsilon').value / msg.info.resolution
        approx = cv2.approxPolyDP(largest, epsilon_px, closed=True)
        if len(approx) < 3:
            return
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        res = msg.info.resolution
        poly = PolygonStamped()
        poly.header.stamp = msg.header.stamp
        poly.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'map'
        for pt in approx:
            col = float(pt[0][0])
            row = float(pt[0][1])
            poly.polygon.points.append(Point32(
                x=origin_x + col * res,
                y=origin_y + row * res,
                z=0.0,
            ))
        normalized = self._normalize_polygon(poly)
        if normalized is None:
            return
        self._polygon_msg = normalized
        self._polygon_echo_pub.publish(self._polygon_msg)
        self._last_preview_valid = False
        self._publish_status(
            'polygon_ready',
            'Map boundary extracted automatically.',
            point_count=len(self._polygon_msg.polygon.points) - 1,
            frame_id=self._polygon_msg.header.frame_id,
        )

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