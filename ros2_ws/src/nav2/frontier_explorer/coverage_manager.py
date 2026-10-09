from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PoseStamped, PolygonStamped
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
    FreeSpaceValidator, directional_coverage, path_from_swaths, validate_forward_path, validate_headland_bounds, work_paths,
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
        self.declare_parameter('open_segments', True)
        self.declare_parameter('work_segment_length_m', 6.0)
        self.declare_parameter('motion_odom_topic', '/odometry/filtered')
        self.declare_parameter('recovery_enabled', True)
        self.declare_parameter('recovery_backup_enabled', False)
        self.declare_parameter('recovery_max_attempts', 2)
        self.declare_parameter('recovery_backup_distance', 0.15)
        self.declare_parameter('recovery_backup_speed', 0.05)
        self.declare_parameter('planning_timeout_sec', 120.0)
        self.declare_parameter('max_path_points', 250000)
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
        self._preview_sections_pub = self.create_publisher(String, '/coverage/preview_sections', _latched_qos)
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
        self._coverage_regions = []
        self._map_details = {}
        self._obstacle_polygons: list[list[tuple[float, float]]] = []
        self._cached_waypoints: list[PoseStamped] = []
        self._cached_path = Path()
        self._cached_preview_details = {}
        self._compute_goal_handle = None
        self._preview_pending = False
        self._preview_cancel = False
        self._preview_started = 0.0
        self._preview_zone = None
        self._pending_execution = None
        self._layout_pool = ThreadPoolExecutor(max_workers=1)
        self._layout_future = None
        self._layout_cancel = Event()
        self._last_map_received = 0.0
        self._execution = CoverageExecution(self, self._publish_status, self._validate_path,
                             self._footprint, self._clearance, self._min_radius,
                             recovery_enabled=bool(self.get_parameter('recovery_enabled').value),
                             recovery_max_attempts=int(self.get_parameter('recovery_max_attempts').value),
                             recovery_backup_distance=float(self.get_parameter('recovery_backup_distance').value),
                             recovery_backup_speed=float(self.get_parameter('recovery_backup_speed').value),
                             recovery_backup_enabled=bool(self.get_parameter('recovery_backup_enabled').value),
                             motion_odom_topic=self.get_parameter('motion_odom_topic').value)
        self._validated_grid = None
        self._validated_path = None
        self._display_path = Path()
        self._display_sections = []
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
        self.create_timer(1.0, self._republish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self._publish_status('idle', 'Coverage manager ready.')

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
                self._start_checked_execution(remaining, resuming=True,
                                              work_sections=self._execution.remaining_work_sections())
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
        self._display_path = Path()
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
        zone = [(point.x, point.y) for point in self._polygon_msg.polygon.points]
        self._preview_zone = (deepcopy(self._custom_zone_points) if self._custom_polygon_active else
                      None if self._coverage_regions else zone)
        regions = deepcopy(self._coverage_regions or [(self._polygon_msg, self._obstacle_polygons)])
        self._preview_cancel = False
        self._layout_cancel.clear()
        self._preview_pending = True
        self._preview_started = time.monotonic()
        spacing = float(self.get_parameter('swath_spacing').value)
        timeout = float(self.get_parameter('planning_timeout_sec').value)
        point_budget = int(self.get_parameter('max_path_points').value)
        use_open_segments = bool(self.get_parameter('open_segments').value)
        work_segment_length = float(self.get_parameter('work_segment_length_m').value)
        selected_zone = deepcopy(self._preview_zone)
        map_details = deepcopy(self._map_details)

        def compute():
            validator = FreeSpaceValidator(grid, self._footprint, self._clearance, selected_zone)
            path = Path(header=header)
            details = dict(map_details, swath_count=0, perimeter_passes=0, connector_count=0,
                           obstacle_detours=0, short_section_count=0, omitted_perimeter_count=0,
                           estimated_covered_area_m2=0.0, estimated_uncovered_area_m2=0.0,
                           edge_segment_count=0, work_segment_count=0, closed_perimeter_count=0,
                           unconnected_segment_count=0, omitted_work_length_m=0.0,
                           work_sections=[],
                           open_segments=use_open_segments,
                           zone_mode='custom' if self._custom_polygon_active else 'map')
            for polygon, holes in regions:
                remaining = timeout - (time.monotonic() - self._preview_started)
                region, metrics = directional_coverage(
                    [(point.x, point.y) for point in polygon.polygon.points], header, validator,
                    self._min_radius, spacing, holes, self._layout_cancel.is_set,
                    planning_timeout=remaining, max_path_points=point_budget - len(path.poses),
                    open_segments=use_open_segments, work_segment_length=work_segment_length)
                offset = len(path.poses)
                details['work_sections'].extend([[start + offset, end + offset]
                                                for start, end in metrics['work_sections']])
                path.poses.extend(region.poses)
                if len(path.poses) > point_budget:
                    raise ValueError('Coverage exceeds its path point budget; no partial route was accepted')
                for key in ('swath_count', 'perimeter_passes', 'connector_count', 'obstacle_detours',
                            'short_section_count', 'omitted_perimeter_count',
                            'edge_segment_count', 'work_segment_count', 'closed_perimeter_count',
                            'unconnected_segment_count', 'omitted_work_length_m',
                            'estimated_covered_area_m2', 'estimated_uncovered_area_m2'):
                    details[key] += metrics[key]
                for key in ('side_margin_m', 'row_end_margin_m', 'perimeter_margin_m'):
                    details[key] = max(details.get(key, 0.0), metrics[key])
            details['path_length_m'] = sum(validate_forward_path(work, self._min_radius)
                                           for work in work_paths(path, details['work_sections']))
            details['transit_mode'] = 'navigate_to_pose'
            details['planning_time_sec'] = time.monotonic() - self._preview_started
            return self._check_preview(path, details)

        self._layout_future = self._layout_pool.submit(compute)
        self._publish_status('planning', 'Computing safe forward work sections with ordinary navigation transfers.')

    @staticmethod
    def _same_map(first, second):
        return (first is not None and second is not None
                and first.header.frame_id == second.header.frame_id
                and first.info.width == second.info.width and first.info.height == second.info.height
                and first.info.resolution == second.info.resolution and first.info.origin == second.info.origin
                and first.data == second.data)

    def _check_preview(self, path, details):
        grid = self._last_map_msg
        if grid is None or time.monotonic() - self._last_map_received > 10.0:
            raise ValueError('A fresh SLAM map is required')
        if grid.header.frame_id != path.header.frame_id:
            raise ValueError('Coverage path and map frames do not match')

        def check_budget():
            if (self._layout_cancel.is_set() or time.monotonic() - self._preview_started
                    > float(self.get_parameter('planning_timeout_sec').value)):
                raise ValueError('Coverage validation canceled or exceeded its planning time budget')

        paths = work_paths(path, details.get('work_sections'))
        details = dict(details, path_length_m=sum(validate_forward_path(work, self._min_radius)
                                                for work in paths))
        zone = None if self._pending_execution is True else self._preview_zone
        validator = FreeSpaceValidator(grid, self._footprint, self._clearance, zone)
        for work in paths:
            validator.check_path(work, canceled=check_budget)
        return path, details, grid

    def _poll_directional_preview(self):
        if self._layout_future is None or not self._layout_future.done():
            return
        future, self._layout_future = self._layout_future, None
        if self._preview_cancel:
            self._preview_pending = False
            self._pending_execution = None
            self._publish_status('canceled', 'Coverage preview canceled.')
            return
        try:
            path, details, grid = future.result()
            if not self._same_map(grid, self._last_map_msg):
                self._layout_future = self._layout_pool.submit(self._check_preview, path, details)
                return
        except Exception as exc:
            self._preview_pending = False
            self._pending_execution = None
            self.get_logger().warn(f'Coverage layout rejected: {exc}')
            self._publish_status('failed', f'Coverage layout rejected: {exc}')
            return
        self._preview_pending = False
        self._cached_path = path
        self._validated_path, self._validated_grid = path, grid
        self._last_preview_valid = True
        self._prepare_display(path, grid.info.resolution, details.get('work_sections'))
        self._path_republish_count = 3
        self._publish_display()
        omitted = details.get('short_section_count', 0) + details.get('omitted_perimeter_count', 0)
        message = 'Coverage route ready; completion refers to the planned route, not all ground area.'
        if 'estimated_uncovered_area_m2' in details:
            message += f" Estimated area outside the {float(self.get_parameter('swath_spacing').value):.2f} m work strip: {details['estimated_uncovered_area_m2']:.2f} m2."
        if details.get('excluded_area_m2', 0.0) > 0.0:
            message += f" Disconnected map area excluded: {details['excluded_area_m2']:.2f} m2."
        if omitted:
            message += f' {omitted} short sections or edge passes could not fit the forward-motion constraints.'
        unconnected = details.get('unconnected_segment_count', 0)
        if unconnected:
            message += f' {unconnected} work segments have no validated forward connection and were omitted.'
        details['partial_coverage'] = bool(omitted or unconnected or details.get('omitted_work_length_m', 0.0) > 1e-6)
        self._cached_preview_details = deepcopy(details)
        if details.get('open_segments'):
            self.get_logger().info(
                f"Open coverage ready: {details.get('work_segment_count', 0)} work segments, "
                f"{unconnected} unconnected, {details.get('omitted_work_length_m', 0.0):.2f} m omitted, "
                f"{details.get('path_length_m', 0.0):.2f} m route")
        self._publish_status('preview_ready', message,
                             waypoint_count=len(path.poses), **details)
        if self._pending_execution is not None:
            resuming, self._pending_execution = self._pending_execution, None
            self._execution.start(path, resuming=resuming, work_sections=details.get('work_sections'))

    @staticmethod
    def _make_display_path(path, resolution):
        from frontier_explorer.coverage_path import angle_difference, pose_xy_yaw
        result = Path(header=deepcopy(path.header))
        if len(path.poses) < 3:
            result.poses = deepcopy(path.poses)
            return result
        result.poses.append(deepcopy(path.poses[0]))
        previous = pose_xy_yaw(path.poses[0].pose)
        for index in range(1, len(path.poses) - 1):
            current = pose_xy_yaw(path.poses[index].pose)
            following = pose_xy_yaw(path.poses[index + 1].pose)
            tangent = math.atan2(following[1] - previous[1], following[0] - previous[0])
            distance = abs((current[0] - previous[0]) * math.sin(tangent)
                           - (current[1] - previous[1]) * math.cos(tangent))
            if (distance > resolution / 4.0 or abs(angle_difference(current[2], previous[2])) > 0.01
                    or abs(angle_difference(following[2], current[2])) > 0.01
                    or math.dist(previous[:2], current[:2]) > 0.3):
                result.poses.append(deepcopy(path.poses[index]))
                previous = current
        result.poses.append(deepcopy(path.poses[-1]))
        return result

    def _prepare_display(self, path, resolution, sections):
        self._display_path = Path(header=deepcopy(path.header))
        self._display_path.header.stamp = self.get_clock().now().to_msg()
        self._display_sections = []
        for work in work_paths(path, sections):
            reduced = self._make_display_path(work, resolution)
            start = len(self._display_path.poses)
            self._display_path.poses.extend(reduced.poses)
            self._display_sections.append([start, len(self._display_path.poses) - 1])

    def _publish_display(self):
        stamp = self._display_path.header.stamp
        self._preview_sections_pub.publish(String(data=json.dumps(
            dict(stamp=[stamp.sec, stamp.nanosec], work_sections=self._display_sections))))
        self._preview_path_pub.publish(self._display_path)

    def destroy_node(self):
        self._layout_cancel.set()
        self._execution.close()
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
            path, sections = path_from_swaths(result.coverage_path,
                                            int(self.get_parameter('max_path_points').value))
            for work in work_paths(path, sections):
                self._validate_path(work)
        except ValueError as exc:
            self._cached_path = Path()
            self._preview_path_pub.publish(self._cached_path)
            self._publish_status('failed', f'Coverage route rejected: {exc}')
            return
        self._cached_path = path
        self._cached_preview_details = dict(swath_count=len(result.coverage_path.swaths),
                            work_sections=sections, transit_mode='navigate_to_pose',
                            path_length_m=sum(validate_forward_path(work, self._min_radius)
                                              for work in work_paths(path, sections)))
        self._path_republish_count = 10

        self._prepare_display(path, self._last_map_msg.info.resolution, sections)
        self._publish_display()
        self._last_preview_valid = True
        self._state = 'preview_ready'
        self._publish_status(
            'preview_ready',
            'Coverage path ready for inspection or execution.',
            swath_count=len(result.coverage_path.swaths),
            waypoint_count=len(self._cached_path.poses),
            path_length_m=self._cached_preview_details['path_length_m'],
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
        self._start_checked_execution(self._cached_path,
                          work_sections=self._cached_preview_details.get('work_sections'))

    def _start_checked_execution(self, path, resuming=False, work_sections=None):
        if (len(path.poses) > 5000
                and (path is not self._validated_path
                     or not self._same_map(self._validated_grid, self._last_map_msg))):
            self._layout_cancel.clear()
            self._preview_cancel = False
            self._preview_pending = True
            self._preview_started = time.monotonic()
            self._pending_execution = resuming
            self._last_preview_valid = False
            self._layout_future = self._layout_pool.submit(
                self._check_preview, path, dict(deepcopy(self._cached_preview_details), work_sections=work_sections))
            self._publish_status('planning', 'Revalidating the remaining route against the current map.')
        else:
            self._execution.start(path, resuming=resuming, work_sections=work_sections)

    def _busy(self):
        return self._preview_pending or self._execution.busy

    def _validate_path(self, path, ingress=False):
        grid = self._last_map_msg
        if grid is None or time.monotonic() - self._last_map_received > 10.0:
            raise ValueError('A fresh SLAM map is required')
        if path.header.frame_id != grid.header.frame_id:
            raise ValueError('Coverage path and map frames do not match')
        if path is self._validated_path and self._same_map(self._validated_grid, grid):
            return
        validate_forward_path(path, self._min_radius)
        zone = None if ingress else self._preview_zone
        FreeSpaceValidator(grid, self._footprint, self._clearance, zone).check_path(path)

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
        self._cached_preview_details = {}
        self._last_preview_valid = False
        self._preview_path_pub.publish(Path())
        self._state = 'idle'
        self._publish_status('idle', 'Coverage state cleared.')

    def _republish_state(self) -> None:
        """1 Hz keepalive: re-publish cached path for up to 10 s after it changes
        (counter-based) so browsers that connect during planning receive it."""
        self._poll_directional_preview()
        if (self._preview_pending and not self._preview_cancel and time.monotonic() - self._preview_started
            > float(self.get_parameter('planning_timeout_sec').value)):
            self._cancel_active_goals()
        if self._cached_path.poses and self._path_republish_count > 0:
            self._publish_display()
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

        self._coverage_regions = []
        self._map_details = {}
        if self.get_parameter('layout_mode').value == 'directional':
            try:
                robot_position = None
                if not self._custom_polygon_active:
                    pose = self._execution.robot_pose(msg.header.frame_id).pose.position
                    robot_position = (pose.x, pose.y)
                self._coverage_regions, self._map_details = self._map_processor.extract_coverage_regions(
                    msg, self._custom_zone_points if self._custom_polygon_active else None, robot_position)
                normalized = self._coverage_regions[0][0] if self._coverage_regions else None
                obstacle_polygons = [hole for _, holes in self._coverage_regions for hole in holes]
            except Exception as exc:
                self._polygon_msg = PolygonStamped()
                self._last_preview_valid = False
                self._publish_status('polygon_invalid', f'Coverage map selection failed: {exc}')
                return False
        else:
            normalized, obstacle_polygons = self._map_processor.extract_boundary_and_obstacles(
                msg, map_morph_close_radius, map_erode_m, map_contour_epsilon,
                obstacle_min_area_m2, obstacle_dilate_m, self._default_frame_id,
                custom_zone_points=self._custom_zone_points if self._custom_polygon_active else None)

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

    def _publish_status(self, state: str, message: str, **extra: Any) -> None:
        self._state = state
        payload = {'state': state, 'message': message}
        payload.update(extra)
        payload['can_execute'] = bool(self._last_preview_valid and self._cached_path.poses and not self._busy())
        if state in ('blocked', 'failed', 'stale_preview', 'no_preview'):
            self.get_logger().warn(f'{state}: {message}')
        self._status_pub.publish(String(data=json.dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoverageManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()