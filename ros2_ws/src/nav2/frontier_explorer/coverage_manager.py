from __future__ import annotations

import json
import math
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Pose, PoseStamped, PolygonStamped
from nav2_msgs.action import FollowPath, BackUp, Spin, Wait
from nav2_msgs.srv import ClearEntireCostmap
from builtin_interfaces.msg import Duration as MsgDuration
from nav_msgs.msg import OccupancyGrid, Path
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

from frontier_explorer.geometry_utils import flip_quaternion, swath_to_waypoints
from frontier_explorer.map_processor import MapProcessor


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
        self.declare_parameter('map_update_min_interval_sec', 5.0)

        self._default_frame_id = self.get_parameter('default_frame_id').value
        self._headland_width = float(self.get_parameter('headland_width').value)
        self._path_continuity_type = self.get_parameter('path_continuity_type').value
        self._path_type = self.get_parameter('path_type').value
        self._turn_point_distance = float(self.get_parameter('turn_point_distance').value)
        self._action_wait_timeout_sec = float(self.get_parameter('action_wait_timeout_sec').value)
        self._allow_execute_without_fresh_preview = bool(
            self.get_parameter('allow_execute_without_fresh_preview').value
        )
        self._map_update_min_interval_sec = float(self.get_parameter('map_update_min_interval_sec').value)

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
        self._cached_waypoints = []
        self._cached_path = Path()
        self._active_path = Path()
        self._compute_goal_handle = None
        self._nav_goal_handle = None
        self._state = 'idle'

        # Recovery state variables
        self._retry_count: int = 0
        self._max_retries: int = 3
        self._in_recovery: bool = False
        self._recovery_goal_handle = None

        # Recovery clients
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        self._backup_client = ActionClient(self, BackUp, 'backup')
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._wait_client = ActionClient(self, Wait, 'wait')
        self._last_preview_valid = False
        # Counts how many more 1 Hz ticks should republish the cached path.
        # Set to 10 whenever the path is updated; decrements to 0 to stop.
        self._path_republish_count: int = 0
        self._last_map_msg = None
        self._last_logged_obstacle_count: int = -1
        # When True, a user-drawn zone is active; _on_map will not overwrite
        # _polygon_msg until the user explicitly clears the zone.
        self._custom_polygon_active: bool = False

        self._last_processed_map_hash = None
        self._last_processed_map_info = None
        self._last_map_processing_time = None

        self._map_processor = MapProcessor(self.get_logger())

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
                self._on_map(self._last_map_msg, force=True)
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
                        wp.pose.orientation = flip_quaternion(wp.pose.orientation)
            self._cached_waypoints = result.nav_path.poses
            self._cached_path = result.nav_path
            self._path_republish_count = 10
        else:
            endpoint_margin = float(self.get_parameter('swath_endpoint_margin').value)
            self._cached_waypoints = swath_to_waypoints(
                result.coverage_path.swaths,
                frame_id,
                self.get_clock().now().to_msg(),
                endpoint_margin
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
            self._path_republish_count = 10

        self._preview_path_pub.publish(self._cached_path)
        self._last_preview_valid = True
        self._state = 'preview_ready'
        self._publish_status(
            'preview_ready',
            'Coverage path ready for inspection or execution.',
            swath_count=len(result.coverage_path.swaths),
            waypoint_count=len(self._cached_waypoints),
        )

    def _start_execution(self) -> None:
        if self._compute_goal_handle or self._nav_goal_handle:
            self._publish_status('busy', 'Coverage manager is already handling a request.')
            return
        # Reset recovery tracking when starting a fresh execution
        self._retry_count = 0
        self._in_recovery = False
        self._recovery_goal_handle = None
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
        self._active_path = Path()
        self._active_path.header.frame_id = self._cached_path.header.frame_id
        self._active_path.header.stamp = stamp
        self._active_path.poses = []
        for wp in self._cached_path.poses:
            wp_copy = PoseStamped()
            wp_copy.header.frame_id = wp.header.frame_id
            wp_copy.header.stamp = stamp
            wp_copy.pose = wp.pose
            self._active_path.poses.append(wp_copy)

        goal = FollowPath.Goal()
        goal.path = self._active_path
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
            if self._retry_count < self._max_retries:
                self._retry_count += 1
                self.get_logger().warn(
                    f'FollowPath execution failed/aborted (status {status}). Triggering recovery attempt {self._retry_count}/{self._max_retries}.'
                )
                self._run_recovery()
            else:
                self.get_logger().error(
                    f'FollowPath execution failed/aborted (status {status}) and max retries ({self._max_retries}) reached. Aborting.'
                )
                self._state = 'failed'
                self._publish_status('failed', f'Coverage execution failed after maximum retries.')
            return

        # Successfully reached the end of the path
        self._retry_count = 0
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
        if self._recovery_goal_handle is not None:
            canceled = True
            self._recovery_goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_done)
            self._recovery_goal_handle = None

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
        self._retry_count = 0
        self._in_recovery = False
        self._recovery_goal_handle = None
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
        current boundary without waiting for the next map update.
        The path is republished for up to 10 s after it changes (counter-based)
        to avoid continuously serialising large Path messages."""
        if self._polygon_msg.polygon.points:
            self._polygon_echo_pub.publish(self._polygon_msg)
        if self._cached_path.poses and self._path_republish_count > 0:
            self._preview_path_pub.publish(self._cached_path)
            self._path_republish_count -= 1

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
        self._cached_waypoints = []
        self._cached_path = Path()
        self._preview_path_pub.publish(Path())

        # Re-run map extraction immediately if a map is available
        if self._last_map_msg is not None:
            self._state = 'idle'
            self._on_map(self._last_map_msg, force=True)

        # If map extraction was not successful (or didn't run), publish the empty boundary to clear it from the UI.
        if not self._polygon_msg.polygon.points:
            self._polygon_echo_pub.publish(self._polygon_msg)
            self._obstacles_pub.publish(String(data='[]'))
            self._publish_status('idle', 'Zone cleared. Waiting for SLAM map.')
        self.get_logger().info('Custom coverage zone cleared; reverted to SLAM boundary.')

    def _on_map(self, msg: OccupancyGrid, force: bool = False) -> None:
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

        # Metadata tuple — cheap to build, used for early-exit guards
        map_info_tuple = (
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y
        )

        map_data_hash = None
        if not force:
            # 1. Rate-limit first — cheapest guard, rejects most callbacks
            current_time = self.get_clock().now()
            if self._last_map_processing_time is not None:
                elapsed = (current_time - self._last_map_processing_time).nanoseconds / 1e9
                if elapsed < self._map_update_min_interval_sec:
                    return  # Rate limit active

            # 2. Only pay for hashing when metadata matches (skips hash on map resize)
            if self._last_processed_map_info == map_info_tuple:
                try:
                    map_data_hash = hash(bytes(msg.data))
                except Exception:
                    map_data_hash = None
                if self._last_processed_map_hash == map_data_hash:
                    return  # Map data and metadata are unchanged

        # Hash not yet computed (force=True or metadata changed) — compute now
        if map_data_hash is None:
            try:
                map_data_hash = hash(bytes(msg.data))
            except Exception:
                map_data_hash = None

        # Proceed to extract
        self._last_processed_map_info = map_info_tuple
        self._last_processed_map_hash = map_data_hash
        self._last_map_processing_time = self.get_clock().now()

        map_morph_close_radius = int(self.get_parameter('map_morph_close_radius').value)
        map_erode_m = float(self.get_parameter('map_erode_m').value)
        map_contour_epsilon = float(self.get_parameter('map_contour_epsilon').value)
        obstacle_min_area_m2 = float(self.get_parameter('obstacle_min_area_m2').value)
        obstacle_dilate_m = float(self.get_parameter('obstacle_dilate_m').value)

        normalized, obstacle_polygons = self._map_processor.extract_boundary_and_obstacles(
            msg,
            map_morph_close_radius,
            map_erode_m,
            map_contour_epsilon,
            obstacle_min_area_m2,
            obstacle_dilate_m,
            self._default_frame_id
        )

        if normalized is None:
            return

        self._polygon_msg = normalized
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

    def _publish_status(self, state: str, message: str, **extra: Any) -> None:
        self._state = state
        payload = {'state': state, 'message': message}
        payload.update(extra)
        self._status_pub.publish(String(data=json.dumps(payload)))

    def _run_recovery(self) -> None:
        self._in_recovery = True
        self._state = 'recovering'
        
        if self._retry_count == 1:
            self._publish_status(
                'recovering',
                f'Recovery attempt {self._retry_count}/{self._max_retries}: Clearing costmaps.'
            )
            self._clear_costmaps_async(self._resume_execution)
        elif self._retry_count == 2:
            self._publish_status(
                'recovering',
                f'Recovery attempt {self._retry_count}/{self._max_retries}: Waiting for obstacle to clear.'
            )
            self._wait_async(2.0, lambda: self._clear_costmaps_async(self._resume_execution))
        elif self._retry_count == 3:
            self._publish_status(
                'recovering',
                f'Recovery attempt {self._retry_count}/{self._max_retries}: Backing up and spinning.'
            )
            # Backup 0.25m, spin 45 degrees (0.78 rad) to look for clearing, clear costmaps, resume
            self._backup_async(
                -0.25,
                0.15,
                lambda: self._spin_async(
                    0.78,
                    lambda: self._clear_costmaps_async(self._resume_execution)
                )
            )

    def _clear_costmaps_async(self, callback) -> None:
        if not self._clear_local_costmap_client.service_is_ready() or not self._clear_global_costmap_client.service_is_ready():
            self.get_logger().warn('Costmap clearing services not ready. Skipping clear.')
            callback()
            return

        req = ClearEntireCostmap.Request()
        pending = 2

        def on_service_done(future):
            nonlocal pending
            try:
                future.result()
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
            pending -= 1
            if pending == 0:
                self.get_logger().info('Local and global costmaps cleared successfully.')
                # Wait 0.5 seconds for the costmaps to update before resuming
                self._create_one_shot_timer(0.5, callback)

        future_local = self._clear_local_costmap_client.call_async(req)
        future_local.add_done_callback(on_service_done)

        future_global = self._clear_global_costmap_client.call_async(req)
        future_global.add_done_callback(on_service_done)

    def _wait_async(self, duration_sec: float, callback) -> None:
        if not self._wait_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Wait action server not available. Skipping.')
            callback()
            return

        goal = Wait.Goal()
        goal.time = MsgDuration()
        goal.time.sec = int(duration_sec)
        goal.time.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        def on_result(future):
            self._recovery_goal_handle = None
            callback()

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Wait action goal rejected.')
                callback()
                return
            self._recovery_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(on_result)

        self._wait_client.send_goal_async(goal).add_done_callback(on_goal_response)

    def _backup_async(self, distance: float, speed: float, callback) -> None:
        if not self._backup_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Backup action server not available. Skipping.')
            callback()
            return

        goal = BackUp.Goal()
        goal.target.x = float(distance)
        goal.speed = float(speed)
        goal.time_allowance = MsgDuration()
        goal.time_allowance.sec = 10

        def on_result(future):
            self._recovery_goal_handle = None
            callback()

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Backup action goal rejected.')
                callback()
                return
            self._recovery_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(on_result)

        self._backup_client.send_goal_async(goal).add_done_callback(on_goal_response)

    def _spin_async(self, target_yaw: float, callback) -> None:
        if not self._spin_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Spin action server not available. Skipping.')
            callback()
            return

        goal = Spin.Goal()
        goal.target_yaw = float(target_yaw)
        goal.time_allowance = MsgDuration()
        goal.time_allowance.sec = 10

        def on_result(future):
            self._recovery_goal_handle = None
            callback()

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Spin action goal rejected.')
                callback()
                return
            self._recovery_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(on_result)

        self._spin_client.send_goal_async(goal).add_done_callback(on_goal_response)

    def _create_one_shot_timer(self, delay_sec: float, callback) -> None:
        timer = None
        def timer_cb():
            nonlocal timer
            if timer is not None:
                timer.destroy()
            callback()
        timer = self.create_timer(delay_sec, timer_cb)

    def _resume_execution(self) -> None:
        self._in_recovery = False

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().error('Could not get robot pose to resume execution. Aborting.')
            self._state = 'failed'
            self._publish_status('failed', 'Could not determine robot pose to resume.')
            return

        poses = self._active_path.poses
        if not poses:
            self.get_logger().error('No active path to resume. Aborting.')
            self._state = 'failed'
            self._publish_status('failed', 'No active path to resume.')
            return

        closest_idx = self._find_closest_waypoint_index(robot_pose, poses)
        self.get_logger().info(f'Resuming execution from waypoint {closest_idx}/{len(poses)}.')

        resumed_path = Path()
        resumed_path.header.frame_id = self._active_path.header.frame_id
        resumed_path.header.stamp = self.get_clock().now().to_msg()
        resumed_path.poses = poses[closest_idx:]

        self._active_path = resumed_path

        if not resumed_path.poses:
            self._state = 'completed'
            self._publish_status('completed', 'Coverage path completed (no remaining waypoints).')
            return

        goal = FollowPath.Goal()
        goal.path = resumed_path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'

        self._state = 'executing'
        self._publish_status(
            'executing',
            f'Resuming coverage path (attempt {self._retry_count + 1}).',
            waypoint_count=len(goal.path.poses),
        )

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback
        )
        send_future.add_done_callback(self._on_execute_goal_response)

    def _find_closest_waypoint_index(self, robot_pose: Pose, poses: list[PoseStamped]) -> int:
        min_dist = float('inf')
        closest_idx = 0
        rx = robot_pose.position.x
        ry = robot_pose.position.y
        for i, ps in enumerate(poses):
            px = ps.pose.position.x
            py = ps.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoverageManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()