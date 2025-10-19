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

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

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

        # --- image stuff ----------------------------------------------------------
        self._bridge = CvBridge()
        self._depth_msg: Optional[Image] = None
        self._conf_msg: Optional[Image] = None
        self._img_lock = threading.Lock()

        self.create_subscription(Image, "depth_image",
                                 self._depth_cb, 10)
        self.create_subscription(Image, "confidence_image",
                                 self._conf_cb, 10)

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
    def _depth_cb(self, msg: Image) -> None:
        with self._img_lock:
            self._depth_msg = msg

    def _conf_cb(self, msg: Image) -> None:
        with self._img_lock:
            self._conf_msg = msg

    def _vin_cb(self, msg: Float32) -> None:
        with self._vin_lock:
            self._vin = float(msg.data)
    # -------- rpm --------
    def _rpm_cb(self, msg: Float32) -> None:
        with self._rpm_lock:
            self._rpm = float(msg.data)

    # ------------------------------------------------------------------------- #
    #                         image helpers (JPEG)                              #
    # ------------------------------------------------------------------------- #
    def _img_to_jpeg(self, msg: Optional[Image],
                     encoding: str) -> bytes:
        if msg is None:
            return self._no_signal()
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, encoding)
            ok, jpeg = cv2.imencode(".jpg", cv_img,
                                    [cv2.IMWRITE_JPEG_QUALITY, 85])
            return jpeg.tobytes() if ok else self._no_signal()
        except Exception:                                    # pragma: no cover
            logger.exception("CV bridge failed")
            return self._no_signal()

    def depth_frame(self) -> bytes:
        with self._img_lock:
            return self._img_to_jpeg(self._depth_msg, "bgr8")

    def confidence_frame(self) -> bytes:
        with self._img_lock:
            return self._img_to_jpeg(self._conf_msg, "mono8")

    def _no_signal(self) -> bytes:
        """Return a black placeholder JPEG."""
        img = np.zeros((240, 320, 3), np.uint8)
        cv2.putText(img, "No Signal", (65, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), 2)
        return cv2.imencode(".jpg", img)[1].tobytes()

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

    # ------------------------------------------------------------------------- #
    #                              telemetry                                    #
    # ------------------------------------------------------------------------- #
    def latest_voltage(self) -> Optional[float]:
        with self._vin_lock:
            return self._vin
    def latest_rpm(self) -> Optional[float]:
        with self._rpm_lock:
            return self._rpm
