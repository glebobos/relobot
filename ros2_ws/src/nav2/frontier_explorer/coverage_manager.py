from __future__ import annotations

import json
import math
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Pose, PoseStamped, PolygonStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path
import cv2
import numpy as np
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
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
        self.declare_parameter('navigate_through_poses_action_name', 'follow_path')
        self.declare_parameter('headland_width', 0.5)
        self.declare_parameter('path_continuity_type', 'CONTINUOUS')
        self.declare_parameter('path_type', 'REEDS_SHEPP')
        self.declare_parameter('turn_point_distance', 0.1)
        self.declare_parameter('action_wait_timeout_sec', 5.0)
        self.declare_parameter('allow_execute_without_fresh_preview', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_contour_epsilon', 0.5)
        self.declare_parameter('map_morph_close_radius', 3)
        self.declare_parameter('map_erode_m', 0.15)
        self.declare_parameter('swath_endpoint_margin', 0.25)
        self.declare_parameter('obstacle_min_area_m2', 0.0004)  # 4 cm²
        self.declare_parameter('obstacle_dilate_m', 0.20)  # safety margin in metres

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
        # Use TRANSIENT_LOCAL (latched) so new subscribers (e.g. a freshly-loaded
        # browser page) immediately receive the last published polygon without
        # needing to wait for the next map update or a manual Refresh Map click.
        _latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._polygon_echo_pub = self.create_publisher(
            PolygonStamped, self.get_parameter('polygon_echo_topic').value, _latched_qos
        )
        # Obstacle polygons published as JSON [[{x,y},...], ...] for the web viewer.
        self._obstacles_pub = self.create_publisher(
            String, '/coverage/obstacles_active', _latched_qos
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
        self._nav_client = ActionClient(self, FollowPath, nav_through_poses_action_name)

        self._polygon_msg = PolygonStamped()
        self._obstacle_polygons: list[list[tuple[float, float]]] = []
        self._cached_waypoints: list = []
        self._cached_path = Path()
        self._compute_goal_handle = None
        self._nav_goal_handle = None
        self._state = 'idle'
        self._last_preview_valid = False
        self._last_map_msg = None
        self._last_logged_obstacle_count: int = -1
        # When True, a user-drawn zone is active; _on_map will not overwrite
        # _polygon_msg until the user explicitly clears the zone.
        self._custom_polygon_active: bool = False

        # Republish the last-good polygon and preview path at 1 Hz so that
        # browser clients that connect/reconnect while planning or executing
        # immediately receive the boundary even when the state gate in _on_map
        # prevents a fresh extraction.  The TRANSIENT_LOCAL polygon publisher
        # normally handles this for a single reconnect, but rosbridge may
        # subscribe with VOLATILE QoS and miss the latched sample; the timer
        # acts as a fallback keepalive.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(1.0, self._republish_state)

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

        if command.startswith('set_zone:'):
            self._on_set_zone(command[len('set_zone:'):])
            return

        if command == 'clear_zone':
            self._on_clear_zone()
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
        # For a user-drawn custom zone: disable headland so swaths fill the
        # rectangle exactly, without any inset margin.
        # For the auto-detected SLAM map: keep headland to avoid driving into walls.
        goal.generate_headland = not self._custom_polygon_active
        goal.generate_route = True
        goal.generate_path = True
        goal.frame_id = self._polygon_msg.header.frame_id or self._default_frame_id

        # polygons[0] = outer field boundary
        polygon = Coordinates()
        for point in self._polygon_msg.polygon.points:
            coordinate = Coordinate()
            coordinate.axis1 = float(point.x)
            coordinate.axis2 = float(point.y)
            polygon.coordinates.append(coordinate)
        goal.polygons.append(polygon)

        # polygons[1..N] = inner obstacle cutouts (voids the robot must avoid)
        for obs_pts in self._obstacle_polygons:
            obs_coords = Coordinates()
            for (ox, oy) in obs_pts:
                c = Coordinate()
                c.axis1 = float(ox)
                c.axis2 = float(oy)
                obs_coords.coordinates.append(c)
            goal.polygons.append(obs_coords)

        if goal.generate_headland:
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
            obstacle_count=len(self._obstacle_polygons),
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
        if result.nav_path.poses:
            robot_pose = self._get_robot_pose()
            if robot_pose is not None:
                start_p = result.nav_path.poses[0].pose.position
                end_p = result.nav_path.poses[-1].pose.position
                rob_p = robot_pose.position
                dist_start = math.hypot(start_p.x - rob_p.x, start_p.y - rob_p.y)
                dist_end = math.hypot(end_p.x - rob_p.x, end_p.y - rob_p.y)
                if dist_end < dist_start:
                    self.get_logger().info(
                        'Robot is closer to the end of the computed coverage path. Reversing path direction.'
                    )
                    result.nav_path.poses.reverse()
                    for wp in result.nav_path.poses:
                        wp.pose.orientation = self._flip_quaternion(wp.pose.orientation)
            self._cached_waypoints = result.nav_path.poses
            self._cached_path = result.nav_path
        else:
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

        self._preview_path_pub.publish(self._cached_path)
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
        if not self._cached_path.poses:
            self._publish_status('no_preview', 'Preview a coverage path before execution.')
            return
        if not self._last_preview_valid and not self._allow_execute_without_fresh_preview:
            self._publish_status('stale_preview', 'Preview the current polygon again before execution.')
            return
        if not self._nav_client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._publish_status('server_unavailable', 'FollowPath action is not available.')
            return

        stamp = self.get_clock().now().to_msg()
        self._cached_path.header.stamp = stamp
        for wp in self._cached_path.poses:
            wp.header.stamp = stamp

        goal = FollowPath.Goal()
        goal.path = self._cached_path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'

        self._state = 'executing'
        self._publish_status(
            'executing',
            'Executing coverage path.',
            waypoint_count=len(goal.path.poses),
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
            distance_remaining=round(float(feedback.distance_to_goal), 2),
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
        self._obstacle_polygons = []
        self._cached_waypoints = []
        self._cached_path = Path()
        self._last_preview_valid = False
        self._preview_path_pub.publish(Path())
        # Do NOT publish an empty PolygonStamped here: doing so would latch an
        # empty message on TRANSIENT_LOCAL, meaning any browser that reconnects
        # after a clear would receive an empty polygon instead of waiting for the
        # next valid extraction.  The preview path is volatile so clearing it is
        # safe; the polygon stays as the last good value until a new one is ready.
        self._state = 'idle'
        self._publish_status('idle', 'Coverage state cleared.')

    def _republish_state(self) -> None:
        """1 Hz keepalive: re-publish the last-good polygon and cached path so
        browsers that connect/reconnect during planning or execution receive the
        current boundary without waiting for the next map update."""
        if self._polygon_msg.polygon.points:
            self._polygon_echo_pub.publish(self._polygon_msg)
        if self._cached_path.poses:
            self._preview_path_pub.publish(self._cached_path)

    def _on_set_zone(self, json_str: str) -> None:
        """Accept a JSON list of {x, y} points from the web UI and use them as
        the active coverage polygon instead of the auto-detected SLAM boundary."""
        try:
            raw = json.loads(json_str)
            if not isinstance(raw, list) or len(raw) < 3:
                raise ValueError('Need at least 3 points')
            # Validate each point is a dict with finite numeric x/y
            for p in raw:
                if not isinstance(p, dict):
                    raise ValueError('Each point must be an object')
                x = float(p['x'])
                y = float(p['y'])
                if not (math.isfinite(x) and math.isfinite(y)):
                    raise ValueError(f'Non-finite coordinate: x={x} y={y}')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._publish_status('error', f'Invalid zone polygon: {exc}')
            return

        poly = PolygonStamped()
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.header.frame_id = self._default_frame_id
        for p in raw:
            poly.polygon.points.append(
                Point32(x=float(p['x']), y=float(p['y']), z=0.0)
            )
        # Ensure polygon is closed (first == last)
        first = poly.polygon.points[0]
        poly.polygon.points.append(Point32(x=first.x, y=first.y, z=0.0))

        self._polygon_msg = poly
        self._obstacle_polygons = []  # no SLAM obstacles for custom zone
        self._custom_polygon_active = True
        self._last_preview_valid = False
        self._polygon_echo_pub.publish(self._polygon_msg)
        self._obstacles_pub.publish(String(data='[]'))
        self._publish_status(
            'polygon_ready',
            'Custom zone set. Click Preview Coverage to plan.',
            point_count=len(raw),
            zone_mode='custom',
        )
        self.get_logger().info(
            f'Custom coverage zone set with {len(raw)} points.'
        )

    def _on_clear_zone(self) -> None:
        """Revert from a custom zone back to SLAM auto-detection."""
        self._custom_polygon_active = False
        self._last_preview_valid = False
        self._polygon_msg = PolygonStamped()
        self._obstacle_polygons = []
        # Re-run map extraction immediately if a map is available
        if self._last_map_msg is not None:
            self._state = 'idle'
            self._on_map(self._last_map_msg)

        # If map extraction was not successful (or didn't run), publish the empty boundary to clear it from the UI.
        if not self._polygon_msg.polygon.points:
            self._polygon_echo_pub.publish(self._polygon_msg)
            self._obstacles_pub.publish(String(data='[]'))
            self._publish_status('idle', 'Zone cleared. Waiting for SLAM map.')
        self.get_logger().info('Custom coverage zone cleared; reverted to SLAM boundary.')

    def _on_map(self, msg: OccupancyGrid) -> None:
        h = msg.info.height
        w = msg.info.width
        if h == 0 or w == 0:
            return
        self._last_map_msg = msg
        # Skip SLAM extraction when a custom zone is active.
        if self._custom_polygon_active:
            return
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
        # At 0.025 m/cell, 16 px ≈ 0.40 m offset before headland is applied.
        # IMPORTANT: snapshot the mask BEFORE erosion — we use this later for
        # obstacle detection. Erosion merges obstacle blobs that are close to
        # walls into the wall region, making them invisible inside the boundary.
        pre_erode_mask = free_mask.copy()
        res = msg.info.resolution
        erode_m = float(self.get_parameter('map_erode_m').value)
        erode_r = max(0, round(erode_m / res))
        if erode_r > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * erode_r + 1, 2 * erode_r + 1)
            )
            free_mask = cv2.erode(free_mask, kernel)
        # Use RETR_CCOMP to retrieve both outer boundary AND inner holes (obstacles).
        # hierarchy shape: [1, N, 4] — [next, prev, first_child, parent]
        contours, hierarchy = cv2.findContours(
            free_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return
        # Outer contours have parent == -1 in hierarchy[0][i][3]
        outer_contours = [
            c for i, c in enumerate(contours) if hierarchy[0][i][3] == -1
        ]
        if not outer_contours:
            return
        largest = max(outer_contours, key=cv2.contourArea)
        original_area = cv2.contourArea(largest)
        # Reject trivially small contours (less than 1 m² of free space)
        min_area_px = 1.0 / (msg.info.resolution ** 2)
        if original_area < min_area_px:
            return
        epsilon_px = self.get_parameter('map_contour_epsilon').value / msg.info.resolution
        approx = cv2.approxPolyDP(largest, epsilon_px, closed=True)
        # Guard against degenerate simplification: if approxPolyDP collapsed the
        # contour to 4 points (a bounding rectangle), the result is useless for
        # coverage.  We apply two independent tests:
        #   1. Area ratio: approx_area >> original_area means the box is larger
        #      than the actual free region (the classic "max rectangle" symptom).
        #   2. Bounding-rect identity: if the 4-point polygon *is* the axis-
        #      aligned bounding box (its area equals the bounding rect area within
        #      5 %), it is degenerate regardless of area ratio — this catches the
        #      case where the free region itself is already roughly rectangular so
        #      original_area ≈ approx_area and the 1.3 × test passes incorrectly.
        # We also catch rotated bounding boxes via cv2.minAreaRect.
        # Retry with progressively smaller epsilons; require >= 5 points so we
        # never accept a plain 4-corner rectangle from a retry either.
        def _is_bounding_rect(poly_pts, contour) -> bool:
            """Return True if poly_pts is effectively the axis-aligned or rotated
            bounding rectangle of contour."""
            if len(poly_pts) != 4:
                return False
            p_area = cv2.contourArea(poly_pts)
            # Axis-aligned bounding rect
            _x, _y, bw, bh = cv2.boundingRect(contour)
            bbox_area = float(bw * bh)
            if bbox_area > 0 and abs(p_area - bbox_area) / bbox_area < 0.05:
                return True
            # Minimum-area rotated rect
            (_, (rw, rh), _) = cv2.minAreaRect(contour)
            min_rect_area = float(rw * rh)
            if min_rect_area > 0 and abs(p_area - min_rect_area) / min_rect_area < 0.05:
                return True
            return False

        if len(approx) <= 4:
            approx_area = cv2.contourArea(approx)
            if approx_area > original_area * 1.05 or len(approx) < 3 or _is_bounding_rect(approx, largest):
                for scale in (0.5, 0.25, 0.1):
                    smaller_eps = epsilon_px * scale
                    retry = cv2.approxPolyDP(largest, smaller_eps, closed=True)
                    retry_area = cv2.contourArea(retry)
                    if len(retry) >= 5 and retry_area <= original_area * 1.05 and not _is_bounding_rect(retry, largest):
                        approx = retry
                        self.get_logger().info(
                            f'approxPolyDP retry with eps={smaller_eps:.1f}px '  # noqa: E501
                            f'gave {len(approx)} points'
                        )
                        break
                else:
                    # All retries still degenerate — skip this map update to
                    # avoid publishing a bogus bounding-rectangle polygon.
                    self.get_logger().warn(
                        'Map contour simplification produced a degenerate polygon; '
                        'skipping polygon update. Try refreshing the map again.'
                    )
                    return
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

        # --- Extract inner obstacle contours (holes inside the free-space mask) ---
        # Obstacles in the SLAM map are non-free blobs fully enclosed in free space.
        # They appear as child contours of the outer boundary in RETR_CCOMP output.
        obstacle_min_px = self.get_parameter('obstacle_min_area_m2').value / (res ** 2)
        obs_dilate_r = max(1, round(self.get_parameter('obstacle_dilate_m').value / res))
        epsilon_px = self.get_parameter('map_contour_epsilon').value / res
        # Build obstacle mask from the PRE-EROSION free mask so that obstacle
        # blobs near walls are not absorbed by the erosion operation.
        # Invert: occupied/unknown cells become white (obstacle candidates).
        obs_mask = cv2.bitwise_not(pre_erode_mask)
        # Restrict to pixels inside the (eroded) outer field boundary only.
        outer_fill = np.zeros_like(obs_mask)
        cv2.drawContours(outer_fill, [largest], -1, 255, thickness=cv2.FILLED)
        obs_mask = cv2.bitwise_and(obs_mask, outer_fill)
        # Dilate each obstacle blob so the cutout has a safety margin matching
        # the robot footprint + inflation layer before passing to F2C.
        if obs_dilate_r > 0:
            dk = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * obs_dilate_r + 1, 2 * obs_dilate_r + 1)
            )
            obs_mask = cv2.dilate(obs_mask, dk)
        obs_contours, _ = cv2.findContours(
            obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        obstacle_polygons: list[list[tuple[float, float]]] = []
        for obs_c in obs_contours:
            if cv2.contourArea(obs_c) < obstacle_min_px:
                continue
            obs_approx = cv2.approxPolyDP(obs_c, epsilon_px, closed=True)
            if len(obs_approx) < 3:
                continue
            pts: list[tuple[float, float]] = []
            for pt in obs_approx:
                col = float(pt[0][0])
                row = float(pt[0][1])
                pts.append((origin_x + col * res, origin_y + row * res))
            # Ensure closed (first == last), as required by ComputeCoveragePath
            if pts[0] != pts[-1]:
                pts.append(pts[0])
            obstacle_polygons.append(pts)
        self._obstacle_polygons = obstacle_polygons
        n = len(obstacle_polygons)
        if n != self._last_logged_obstacle_count:
            self.get_logger().info(
                f'Obstacle cutouts updated: {n} found in map.'
            )
            self._last_logged_obstacle_count = n
        # Publish obstacle polygons as JSON for the web map viewer.
        # Format: [[{"x": float, "y": float}, ...], ...]  — one list per obstacle.
        obs_json = json.dumps([
            [{'x': x, 'y': y} for (x, y) in poly]
            for poly in obstacle_polygons
        ])
        self._obstacles_pub.publish(String(data=obs_json))

        self._polygon_echo_pub.publish(self._polygon_msg)
        self._last_preview_valid = False
        self._publish_status(
            'polygon_ready',
            'Map boundary extracted automatically.',
            point_count=len(self._polygon_msg.polygon.points) - 1,
            frame_id=self._polygon_msg.header.frame_id,
            obstacle_count=len(self._obstacle_polygons),
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

    def _get_robot_pose(self) -> Pose | None:
        target_frame = self._polygon_msg.header.frame_id or self._default_frame_id
        try:
            t = self._tf_buffer.lookup_transform(
                target_frame,
                'base_link',
                rclpy.time.Time()
            )
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(
                f'Could not look up transform from {target_frame} to base_link: {e}'
            )
            return None

    def _flip_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        new_yaw = yaw + math.pi
        
        from geometry_msgs.msg import Quaternion
        q_new = Quaternion()
        q_new.x = 0.0
        q_new.y = 0.0
        q_new.z = math.sin(new_yaw / 2.0)
        q_new.w = math.cos(new_yaw / 2.0)
        return q_new

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