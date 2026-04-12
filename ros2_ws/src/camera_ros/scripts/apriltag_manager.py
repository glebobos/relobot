#!/usr/bin/env python3
"""On-demand AprilTag loader for dock actions."""

import os
import subprocess
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg import GoalStatusArray, GoalStatus
from composition_interfaces.srv import LoadNode, UnloadNode
from rcl_interfaces.msg import (
    Parameter as ParameterMsg,
    ParameterValue,
    ParameterType,
)
from ament_index_python.packages import get_package_share_directory

ACTIVE_STATUSES = frozenset({
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
})


# ------------------------------------------------------------------
# Pure helpers (module-level, stateless)
# ------------------------------------------------------------------
def _flatten_dict(d: dict, prefix: str = '', sep: str = '.') -> dict:
    items: dict = {}
    for k, v in d.items():
        key = f'{prefix}{sep}{k}' if prefix else k
        if isinstance(v, dict):
            items.update(_flatten_dict(v, key, sep))
        else:
            items[key] = v
    return items


def _make_param(name: str, val) -> ParameterMsg:
    p = ParameterMsg(name=name, value=ParameterValue())
    if isinstance(val, bool):
        p.value.type = ParameterType.PARAMETER_BOOL
        p.value.bool_value = val
    elif isinstance(val, int):
        p.value.type = ParameterType.PARAMETER_INTEGER
        p.value.integer_value = val
    elif isinstance(val, float):
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = val
    elif isinstance(val, str):
        p.value.type = ParameterType.PARAMETER_STRING
        p.value.string_value = val
    else:
        raise TypeError(f"Unsupported type for '{name}': {type(val)}")
    return p


def _load_yaml_params(package: str, rel_path: str, root_key: str):
    path = os.path.join(get_package_share_directory(package), rel_path)
    with open(path) as f:
        raw = yaml.safe_load(f)[root_key]['ros__parameters']
    return [_make_param(k, v) for k, v in _flatten_dict(raw).items()]


# ------------------------------------------------------------------
class AprilTagManager(Node):

    def __init__(self):
        super().__init__('apriltag_manager')

        # --- ROS parameters (replaces env-var) ---
        self.declare_parameter('container', '/camera_container')
        self.declare_parameter('dock_action', '/dock_robot')
        self.declare_parameter('dock_tag_frame', 'tag25h9:0')
        self.declare_parameter('always_on', False)

        container = self.get_parameter('container').value
        dock_action = self.get_parameter('dock_action').value
        self._tag_frame = self.get_parameter('dock_tag_frame').value
        self._always_on = self.get_parameter('always_on').value

        # --- state ---
        self._lock = threading.Lock()
        self._apriltag_uid: int | None = None
        self._dock_pose_proc: subprocess.Popen | None = None
        self._loading = False
        self._active_goals: set[tuple] = set()
        
        self._retry_timer = None

        # --- load YAML once at startup ---
        try:
            self._apriltag_params = _load_yaml_params(
                'camera_ros', 'config/apriltag.yaml', 'apriltag')
        except Exception as e:
            self.get_logger().fatal(f'Cannot load apriltag.yaml: {e}')
            raise

        # --- composition clients ---
        cb = MutuallyExclusiveCallbackGroup()
        self._load_cli = self.create_client(
            LoadNode, f'{container}/_container/load_node', callback_group=cb)
        self._unload_cli = self.create_client(
            UnloadNode, f'{container}/_container/unload_node', callback_group=cb)

        # --- kick-off ---
        if self._always_on:
            self.get_logger().info('always_on — will load AprilTag permanently')
            self._retry_timer = self.create_timer(1.0, self._try_load)
        else:
            self.create_subscription(
                GoalStatusArray, f'{dock_action}/_action/status',
                self._on_dock_status, 10)
            self.get_logger().info('Waiting for dock goals')

    # ------------------------------------------------------------------
    def _try_load(self):
        """Retry-safe wrapper."""
        if self._load_cli.service_is_ready():
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
            self._load_apriltag()

    # ------------------------------------------------------------------
    def _on_dock_status(self, msg: GoalStatusArray):
        current = {
            tuple(s.goal_info.goal_id.uuid)
            for s in msg.status_list
            if s.status in ACTIVE_STATUSES
        }
        was, now = bool(self._active_goals), bool(current)
        self._active_goals = current

        if now and not was:
            self._load_apriltag()
        elif not now and was:
            self._unload_apriltag()

    # ------------------------------------------------------------------
    def _load_apriltag(self):
        with self._lock:
            if self._apriltag_uid is not None or self._loading:
                return
            if not self._load_cli.service_is_ready():
                self.get_logger().warn('LoadNode not ready — retrying in 1 s')
                # Start a timer only if one is not already running
                if self._retry_timer is None:
                    self._retry_timer = self.create_timer(1.0, self._try_load)
                return
            self._loading = True

        self.get_logger().info('Loading AprilTag …')
        req = LoadNode.Request()
        req.package_name = 'apriltag_ros'
        req.plugin_name = 'AprilTagNode'
        req.node_name = 'apriltag'
        req.remap_rules = [
            'image_rect:=/camera/image_raw',
            'camera_info:=/camera/camera_info',
        ]
        req.parameters = self._apriltag_params
        req.extra_arguments = [
            rclpy.parameter.Parameter(
                'use_intra_process_comms', value=True,
            ).to_parameter_msg(),
        ]
        self._load_cli.call_async(req).add_done_callback(self._on_load_done)

    def _on_load_done(self, future):
        with self._lock:
            self._loading = False
            
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'LoadNode call failed: {e}')
            return
            
        if not res.success:
            self.get_logger().error(f'LoadNode rejected: {res.error_message}')
            return
            
        with self._lock:
            self._apriltag_uid = res.unique_id
            
        self.get_logger().info(f'AprilTag loaded (uid={res.unique_id})')
        self._start_dock_pose_publisher()
        
        # [CRITICAL FIX] Handle race condition: goal ended while we were busy loading
        if not self._always_on and not self._active_goals:
            self.get_logger().info('Dock goal finished during load; unloading immediately.')
            self._unload_apriltag()

    # ------------------------------------------------------------------
    def _unload_apriltag(self):
        self._stop_dock_pose_publisher()
        
        with self._lock:
            uid = self._apriltag_uid
            self._apriltag_uid = None
            
        if uid is None:
            return
            
        if not self._unload_cli.service_is_ready():
            self.get_logger().error('UnloadNode service unavailable')
            return
            
        self.get_logger().info('Unloading AprilTag …')
        self._unload_cli.call_async(
            UnloadNode.Request(unique_id=uid),
        ).add_done_callback(self._on_unload_done)

    def _on_unload_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info('AprilTag unloaded')
            else:
                self.get_logger().error(f'Unload failed: {res.error_message}')
        except Exception as e:
            self.get_logger().error(f'UnloadNode call failed: {e}')

    # ------------------------------------------------------------------
    def _start_dock_pose_publisher(self):
        if self._dock_pose_proc is not None:
            return
        self.get_logger().info('Starting dock_pose_publisher')
        try:
            self._dock_pose_proc = subprocess.Popen(
                ['ros2', 'run', 'camera_ros', 'dock_pose_publisher',
                 '--ros-args',
                 '-p', f'dock_tag_frame:={self._tag_frame}',
                 '-p', 'base_frame:=odom'],
                start_new_session=True,
            )
        except Exception as e:
            self.get_logger().error(f'dock_pose_publisher start failed: {e}')

    def _stop_dock_pose_publisher(self):
        proc = self._dock_pose_proc
        if proc is None:
            return
        self._dock_pose_proc = None
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        self.get_logger().info('dock_pose_publisher stopped')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._unload_apriltag()
        node._stop_dock_pose_publisher()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
