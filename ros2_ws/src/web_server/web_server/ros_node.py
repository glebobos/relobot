# Copyright 2025 ReloBot Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Thin ROS2 wrapper that:
  • publishes Twist + Float32,
  • exposes helpers to retrieve camera frames + voltage,
  • periodically runs GC to avoid leaks in long-running Flask procs.
"""

import gc
import logging
import threading
from typing import Optional
# image handling removed: depth/confidence frames are not used in the web server
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from slam_toolbox.srv import SerializePoseGraph
from opennav_docking_msgs.action import DockRobot, UndockRobot

logger = logging.getLogger(__name__)


class RobotROSNode(Node):

    def __init__(self) -> None:
        super().__init__("robot_web_server_node")

        # --- pubs ----------------------------------------------------------------
        self._vel_pub = self.create_publisher(
            Twist, "/cmd_vel", 10)
        self._knife_pub = self.create_publisher(Float32, "knives/set_rpm", 10)

        # --- service client -------------------------------------------------------
        self._pid_client = self.create_client(SetBool, "knives/enable_pid")
        self._map_saver_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

        # --- action clients -------------------------------------------------------
        self._dock_action_client = ActionClient(self, DockRobot, 'dock_robot')
        self._undock_action_client = ActionClient(self, UndockRobot, 'undock_robot')

    # image subscriptions removed (depth/confidence frames not used)

        # --- voltage --------------------------------------------------------------
        self._vin: Optional[float] = None
        self._vin_lock = threading.Lock()
        self.create_subscription(Float32, "knives/vin",
                                 self._vin_cb, 10)
        # --- RPM ------------------------------------------------------------------
        self._rpm: Optional[float] = None
        self._rpm_lock = threading.Lock()
        self.create_subscription(Float32, "knives/current_rpm",
                                 self._rpm_cb, 10)

        # --- maintenance ----------------------------------------------------------
        self.create_timer(10.0, lambda: gc.collect())
        self.get_logger().info("ROS node ready")

    # ------------------------------------------------------------------------- #
    #                               callbacks                                    #
    # ------------------------------------------------------------------------- #
    # depth/confidence callbacks removed

    def _vin_cb(self, msg: Float32) -> None:
        with self._vin_lock:
            self._vin = float(msg.data)
    # -------- rpm --------
    def _rpm_cb(self, msg: Float32) -> None:
        with self._rpm_lock:
            self._rpm = float(msg.data)

    # image helpers removed

    # ------------------------------------------------------------------------- #
    #                        publishing wrappers                                #
    # ------------------------------------------------------------------------- #
    def publish_twist(self, lin: float, ang: float) -> None:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self._vel_pub.publish(msg)

    def publish_rpm(self, rpm: float) -> None:
        self._knife_pub.publish(Float32(data=rpm))

    def toggle_pid(self, enable: bool) -> None:
        if not self._pid_client.service_is_ready():
            self.get_logger().warning("PID service unavailable")
            return
        req = SetBool.Request(data=enable)
        self._pid_client.call_async(req)   # we ignore response here

    def save_map(self, filename: str) -> None:
        if not self._map_saver_client.service_is_ready():
            self.get_logger().warning("Map Saver service unavailable")
            return
        
        req = SerializePoseGraph.Request()
        req.filename = filename 
        self._map_saver_client.call_async(req)

    def dock_robot(self, dock_id: str = 'home_dock', navigate_to_staging: bool = True) -> bool:
        """Send dock robot action request."""
        self.get_logger().info(f"Attempting to dock at: {dock_id}")
        if not self._dock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Dock action server unavailable after 5s timeout")
            return False
        
        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.navigate_to_staging_pose = navigate_to_staging
        goal_msg.max_staging_time = 60.0
        
        self.get_logger().info(f"Sending dock request to: {dock_id}")
        future = self._dock_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Dock goal sent successfully")
        return True

    def undock_robot(self) -> bool:
        """Send undock robot action request."""
        self.get_logger().info("Attempting to undock")
        if not self._undock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Undock action server unavailable after 5s timeout")
            return False
        
        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = 'apriltag_dock'
        goal_msg.max_undocking_time = 30.0
        
        self.get_logger().info("Sending undock request")
        future = self._undock_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Undock goal sent successfully")
        return True

    # ------------------------------------------------------------------------- #
    #                              telemetry                                    #
    # ------------------------------------------------------------------------- #
    def latest_voltage(self) -> Optional[float]:
        with self._vin_lock:
            return self._vin
    def latest_rpm(self) -> Optional[float]:
        with self._rpm_lock:
            return self._rpm
