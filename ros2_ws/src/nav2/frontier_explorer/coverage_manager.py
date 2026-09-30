from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Pose, PoseStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Path
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

from frontier_explorer.coverage_execution import CoverageExecution
from frontier_explorer.coverage_path import (
    FreeSpaceValidator, directional_coverage, validate_forward_path, validate_headland_bounds,
)
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
        self.declare_parameter('headland_width', 1.5)
        self.declare_parameter('layout_mode', 'fields2cover')
        self.declare_parameter('swath_spacing', 0.24)
        self.declare_parameter('path_continuity_type', 'DISCONTINUOUS')
        self.declare_parameter('path_type', 'DUBIN')
        self.declare_parameter('turn_point_distance', 0.03)
        self.declare_parameter('min_turning_radius', 0.20)
        self.declare_parameter('clearance', 0.20)
        self.declare_parameter('footprint', '[[-0.10,-0.245],[-0.10,0.175],[0.51,0.175],[0.51,-0.245]]')
        self.declare_parameter('action_wait_timeout_sec', 5.0)
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
        self._footprint = json.loads(self.get_parameter('footprint').value)
        self._clearance = float(self.get_parameter('clearance').value)
        self._min_radius = float(self.get_parameter('min_turning_radius').value)

        compute_action_name = self.get_parameter('compute_coverage_action_name').value

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
            _latched_qos,
        )

        self._compute_client = ActionClient(self, ComputeCoveragePath, compute_action_name)

        self._polygon_msg = PolygonStamped()
        self._obstacle_polygons: list[list[tuple[float, float]]] = []
        self._cached_waypoints: list[PoseStamped] = []
        self._cached_path = Path()
        self._compute_goal_handle = None
        self._preview_pending = False
        self._preview_cancel = False
        self._preview_started = 0.0
        self._preview_zone = None
        self._layout_pool = ThreadPoolExecutor(max_workers=1)
        self._layout_future = None
        self._layout_cancel = Event()
        self._last_map_received = 0.0
        self._execution = CoverageExecution(self, self._publish_status, self._validate_path,
                             self._footprint, self._clearance, self._min_radius)
        self._state = 'idle'
        self._last_preview_valid = False
        # Counts how many more 1 Hz ticks should republish the cached path.
        # Set to 10 whenever the path is updated; decrements to 0 to stop.
        self._path_republish_count: int = 0
        self._last_map_msg = None
        self._last_logged_obstacle_count: int = -1
        # When True, a user-drawn zone is active; _on_map will not overwrite
        # _polygon_msg until the user explicitly clears the zone.
        self._custom_polygon_active: bool = False
        self._custom_zone_points: list[tuple[float, float]] | None = None

        self._map_processor = MapProcessor(self.get_logger())

        # Republish the last-good polygon and preview path at 1 Hz so that
        # browser clients that connect/reconnect while planning or executing
        # immediately receive the boundary even when the state gate in _on_map
        # prevents a fresh extraction.  The TRANSIENT_LOCAL polygon publisher
        # normally handles this for a single reconnect, but rosbridge may
        # subscribe with VOLATILE QoS and miss the latched sample; the timer
        # acts as a fallback keepalive.
        self._last_robot_pose: Pose | None = None
        self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self._on_robot_pose,
            1,
        )

        self.create_timer(1.0, self._republish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self._publish_status('idle', 'Coverage manager ready.')

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        self._last_robot_pose = msg.pose

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if self._busy() and command not in {'cancel', 'stop'}:
            self._publish_status('busy', 'Cancel the active coverage request before changing it.')
            return
        if command == 'preview':
            self._start_preview()
            return
        if command == 'execute':
            self._start_execution()
            return
        if command == 'resume':
            remaining = self._execution.remaining_path()
            if remaining is None or self._execution.phase not in {'blocked', 'canceled'}:
                self._publish_status('failed', 'No interrupted route is available to resume.')
            else:
                self._execution.start(remaining, resuming=True)
            return
        if command in {'cancel', 'stop'}:
            self._cancel_active_goals()
            return
        if command == 'clear':
            self._clear_cached_state()
            return
        if command == 'refresh_map':
            if self._last_map_msg is not None:
                self._execution.forget()
                self._last_preview_valid = False
                self._extract_map_boundary(force=True)
            else:
                self._publish_status('error', 'No map received yet.')
            return

        if command.startswith('set_zone:'):
            self._on_set_zone(command[len('set_zone:'):])
            return

        if command == 'clear_zone':
            self._on_clear_zone()
            return
        if command == 'restart_slam':
            self._restart_slam()
            return
        if command == 'reset_map':
            self._reset_map()
            return

        self._publish_status('error', f'Unknown coverage command: {command}')

    def _restart_slam(self) -> None:
        self.get_logger().info('Restarting SLAM toolbox by terminating the nav2 launch process...')
        import subprocess
        try:
            # Terminate the ros2 launch process gracefully, allowing child nodes to clean up
            subprocess.run(['pkill', '-f', 'ros2 launch nav2'], check=True)
        except Exception as e:
            self.get_logger().error(f'Failed to kill launch process with pkill: {e}')
            import os
            import signal
            try:
                os.kill(1, signal.SIGTERM)
            except Exception as ex:
                self.get_logger().error(f'Failed to kill PID 1: {ex}')

    def _reset_map(self) -> None:
        self.get_logger().info('Resetting map (deleting serialized pose graph and data files)...')
        import os
        for ext in ('.posegraph', '.data'):
            path = '/ros2_ws/map_serialized' + ext
            if os.path.isfile(path):
                try:
                    os.remove(path)
                    self.get_logger().info(f'Successfully deleted: {path}')
                except Exception as e:
                    self.get_logger().error(f'Failed to delete {path}: {e}')
            else:
                self.get_logger().info(f'Map file does not exist: {path}')
        self._restart_slam()


    def _start_preview(self) -> None:
        if self._busy():
            self._publish_status('busy', 'Coverage manager is already handling a request.')
            return
        if len(self._polygon_msg.polygon.points) < 4:
            if not self._extract_map_boundary(force=True):
                self._publish_status('polygon_invalid', 'No map boundary yet. Wait for SLAM map or click Refresh Map.')
                return
        if len(self._polygon_msg.polygon.points) < 4:
            self._publish_status('polygon_invalid', 'No map boundary yet. Wait for SLAM map or click Refresh Map.')
            return
        if self.get_parameter('layout_mode').value == 'directional':
            self._start_directional_preview()
            return
        try:
            validate_headland_bounds(
                [(point.x, point.y) for point in self._polygon_msg.polygon.points], self._headland_width)
        except ValueError as exc:
            self._last_preview_valid = False
            self._cached_path = Path()
            self._preview_path_pub.publish(Path())
            self._publish_status('polygon_invalid', str(exc))
            return
        if not self._compute_client.server_is_ready():
            self._publish_status('server_unavailable', 'Coverage server action is not available.')
            return

        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = True
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
        goal.route_mode.mode = 'SNAKE'
        self._preview_zone = [(point.x, point.y) for point in self._polygon_msg.polygon.points]
        self._last_preview_valid = False
        self._preview_pending = True
        self._preview_cancel = False
        self._execution.forget()
        self._preview_started = time.monotonic()

        self._state = 'planning'
        self._publish_status(
            'planning',
            'Computing coverage path.',
            frame_id=goal.frame_id,
            point_count=len(self._polygon_msg.polygon.points) - 1,
            obstacle_count=len(self._obstacle_polygons),
        )

        try:
            send_future = self._compute_client.send_goal_async(goal)
            send_future.add_done_callback(self._on_preview_goal_response)
        except Exception as exc:
            self._preview_pending = False
            self._publish_status('failed', f'Coverage request failed: {exc}')

    def _start_directional_preview(self):
        self._last_preview_valid = False
        self._cached_path = Path()
        self._preview_path_pub.publish(Path())
        self._execution.forget()
        if self._last_map_msg is None or time.monotonic() - self._last_map_received > 10.0:
            self._publish_status('failed', 'A fresh SLAM map is required')
            return
        grid = deepcopy(self._last_map_msg)
        header = deepcopy(self._polygon_msg.header)
        if header.frame_id != grid.header.frame_id:
            self._publish_status('failed', 'Coverage polygon and map frames do not match')
            return
        self._preview_zone = [(point.x, point.y) for point in self._polygon_msg.polygon.points]
        zone, holes = deepcopy(self._preview_zone), deepcopy(self._obstacle_polygons)
        self._preview_cancel = False
        self._layout_cancel.clear()
        self._preview_pending = True
        self._preview_started = time.monotonic()
        spacing = float(self.get_parameter('swath_spacing').value)

        def compute():
            validator = FreeSpaceValidator(grid, self._footprint, self._clearance, zone)
            return directional_coverage(zone, header, validator, self._min_radius, spacing,
                                         holes, self._layout_cancel.is_set)

        self._layout_future = self._layout_pool.submit(compute)
        self._publish_status('planning', 'Computing directional rows and a rounded perimeter pass.')

    def _poll_directional_preview(self):
        if self._layout_future is None or not self._layout_future.done():
            return
        future, self._layout_future = self._layout_future, None
        self._preview_pending = False
        if self._preview_cancel:
            self._publish_status('canceled', 'Coverage preview canceled.')
            return
        try:
            path, details = future.result()
            self._validate_path(path)
        except Exception as exc:
            self._publish_status('failed', f'Coverage layout rejected: {exc}')
            return
        self._cached_path = path
        self._last_preview_valid = True
        self._path_republish_count = 10
        self._preview_path_pub.publish(path)
        self._publish_status('preview_ready', 'Directional rows and perimeter ready for inspection.',
                             waypoint_count=len(path.poses), **details)

    def destroy_node(self):
        self._layout_cancel.set()
        self._layout_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _on_preview_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._preview_pending = False
            self._compute_goal_handle = None
            self._publish_status('error', f'Coverage preview request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._preview_pending = False
            self._compute_goal_handle = None
            self._publish_status('rejected', 'Coverage preview request was rejected.')
            return

        self._compute_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_preview_result)
        if self._preview_cancel:
            goal_handle.cancel_goal_async()

    def _on_preview_result(self, future) -> None:
        self._compute_goal_handle = None
        self._preview_pending = False
        if self._preview_cancel:
            self._publish_status('canceled', 'Coverage preview canceled.')
            return
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

        try:
            if not result.coverage_path.swaths_ordered or not result.coverage_path.contains_turns:
                raise ValueError('Coverage server did not return ordered swaths with turn geometry')
            self._validate_path(result.nav_path)
        except ValueError as exc:
            self._cached_path = Path()
            self._preview_path_pub.publish(self._cached_path)
            self._publish_status('failed', f'Coverage route rejected: {exc}')
            return
        self._cached_path = deepcopy(result.nav_path)
        self._path_republish_count = 10

        self._preview_path_pub.publish(self._cached_path)
        self._last_preview_valid = True
        self._state = 'preview_ready'
        self._publish_status(
            'preview_ready',
            'Coverage path ready for inspection or execution.',
            swath_count=len(result.coverage_path.swaths),
            waypoint_count=len(self._cached_path.poses),
            path_length_m=validate_forward_path(self._cached_path, self._min_radius),
        )

    def _start_execution(self) -> None:
        if self._busy():
            self._publish_status('busy', 'Coverage manager is already handling a request.')
            return
        if not self._cached_path.poses:
            self._publish_status('no_preview', 'Preview a coverage path before execution.')
            return
        if not self._last_preview_valid:
            self._publish_status('stale_preview', 'Preview the current polygon again before execution.')
            return
        self._execution.start(self._cached_path)

    def _busy(self):
        return self._preview_pending or self._execution.busy

    def _validate_path(self, path, ingress=False):
        if self._last_map_msg is None or time.monotonic() - self._last_map_received > 10.0:
            raise ValueError('A fresh SLAM map is required')
        if path.header.frame_id != self._last_map_msg.header.frame_id:
            raise ValueError('Coverage path and map frames do not match')
        validate_forward_path(path, self._min_radius)
        zone = None if ingress else self._preview_zone
        FreeSpaceValidator(self._last_map_msg, self._footprint, self._clearance, zone).check_path(path)

    def _cancel_active_goals(self) -> None:
        if self._preview_pending:
            self._preview_cancel = True
            self._layout_cancel.set()
            self._publish_status('cancel_requested', 'Canceling coverage preview.')
            if self._compute_goal_handle:
                self._compute_goal_handle.cancel_goal_async()
        elif self._execution.busy:
            self._execution.cancel()
        else:
            self._publish_status('idle', 'No active coverage task to cancel.')

    def _clear_cached_state(self) -> None:
        self._cancel_active_goals()
        self._execution.forget()
        self._polygon_msg = PolygonStamped()
        self._obstacle_polygons = []
        self._cached_waypoints = []
        self._cached_path = Path()
        self._last_preview_valid = False
        self._preview_path_pub.publish(Path())
        self._state = 'idle'
        self._publish_status('idle', 'Coverage state cleared.')

    def _republish_state(self) -> None:
        """1 Hz keepalive: re-publish cached path for up to 10 s after it changes
        (counter-based) so browsers that connect during planning receive it."""
        self._poll_directional_preview()
        if self._preview_pending and not self._preview_cancel and time.monotonic() - self._preview_started > 30.0:
            self._cancel_active_goals()
        if self._cached_path.poses and self._path_republish_count > 0:
            self._preview_path_pub.publish(self._cached_path)
            self._path_republish_count -= 1

    def _on_set_zone(self, json_str: str) -> None:
        """Accept a JSON list of {x, y} points from the web UI and use them as
        the active coverage polygon boundary, intersected with the SLAM map."""
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

        self._custom_zone_points = [(float(p['x']), float(p['y'])) for p in raw]
        self._execution.forget()
        self._polygon_msg = PolygonStamped()
        self._obstacle_polygons = []
        self._cached_path = Path()
        self._preview_path_pub.publish(Path())
        self._custom_polygon_active = True
        self._last_preview_valid = False

        # Re-run map extraction immediately if a map is available
        if self._last_map_msg is not None:
            self._extract_map_boundary(force=True)
        else:
            poly = PolygonStamped()
            poly.header.stamp = self.get_clock().now().to_msg()
            poly.header.frame_id = self._default_frame_id
            for p in raw:
                poly.polygon.points.append(
                    Point32(x=float(p['x']), y=float(p['y']), z=0.0)
                )
            first = poly.polygon.points[0]
            poly.polygon.points.append(Point32(x=first.x, y=first.y, z=0.0))

            self._polygon_msg = poly
            self._obstacle_polygons = []
            self._polygon_echo_pub.publish(self._polygon_msg)
            self._obstacles_pub.publish(String(data='[]'))
            self._publish_status(
                'polygon_ready',
                'Custom zone set. Waiting for map to intersect.',
                point_count=len(raw),
                zone_mode='custom',
            )

        self.get_logger().info(
            f'Custom coverage zone set with {len(raw)} points.'
        )

    def _on_clear_zone(self) -> None:
        """Revert from a custom zone back to SLAM auto-detection."""
        self._execution.forget()
        self._custom_polygon_active = False
        self._custom_zone_points = None
        self._last_preview_valid = False
        self._polygon_msg = PolygonStamped()
        self._obstacle_polygons = []
        self._cached_waypoints = []
        self._cached_path = Path()
        self._preview_path_pub.publish(Path())

        # Re-run map extraction immediately if a map is available
        if self._last_map_msg is not None:
            self._extract_map_boundary(force=True)
        else:
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
        self._last_map_received = time.monotonic()

    def _extract_map_boundary(self, force: bool = False) -> bool:
        if self._last_map_msg is None:
            self._publish_status('error', 'No map received yet.')
            return False

        msg = self._last_map_msg
        h = msg.info.height
        w = msg.info.width
        if h == 0 or w == 0:
            return False

        # Only re-extract polygon when idle or ready — don't overwrite state during
        # planning, preview_ready, executing, or completed.
        if not force and self._state not in ('idle', 'polygon_ready'):
            return False

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
            self._default_frame_id,
            custom_zone_points=self._custom_zone_points if self._custom_polygon_active else None
        )

        if normalized is None:
            self._polygon_msg = PolygonStamped()
            self._obstacle_polygons = []
            self._last_preview_valid = False
            if self._custom_polygon_active:
                self._publish_status(
                    'polygon_invalid',
                    'No free space inside the custom zone. Adjust the rectangle.',
                    zone_mode='custom'
                )
            else:
                self._publish_status(
                    'polygon_invalid',
                    'Could not extract boundary from current map.',
                )
            return False

        self._polygon_msg = normalized
        self._obstacle_polygons = obstacle_polygons

        n = len(obstacle_polygons)
        if n != self._last_logged_obstacle_count:
            self.get_logger().info(
                f'Obstacle cutouts updated: {n} found in map.'
            )
            self._last_logged_obstacle_count = n

        # Publish obstacle polygons as JSON for the web map viewer.
        # Format: [[{"x": float, "y": float}, ...], ...] — one list per obstacle.
        obs_json = json.dumps([
            [{'x': x, 'y': y} for (x, y) in poly]
            for poly in obstacle_polygons
        ])
        self._obstacles_pub.publish(String(data=obs_json))
        self._polygon_echo_pub.publish(self._polygon_msg)
        self._last_preview_valid = False

        if self._custom_polygon_active:
            self._publish_status(
                'polygon_ready',
                'Custom zone set and map intersected. Click Preview Coverage to plan.',
                point_count=len(self._polygon_msg.polygon.points) - 1,
                frame_id=self._polygon_msg.header.frame_id,
                obstacle_count=len(self._obstacle_polygons),
                zone_mode='custom'
            )
        else:
            self._publish_status(
                'polygon_ready',
                'Map boundary extracted.',
                point_count=len(self._polygon_msg.polygon.points) - 1,
                frame_id=self._polygon_msg.header.frame_id,
                obstacle_count=len(self._obstacle_polygons),
            )
        return True

    def _get_robot_pose(self) -> Pose | None:
        if self._last_robot_pose is not None:
            return self._last_robot_pose
        self.get_logger().warn('No /robot_pose received yet')
        return None

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