"""
Thin ROS2 wrapper that:
  • publishes Twist + Float32,
  • exposes helpers to retrieve camera frames + voltage,
  • periodically runs GC to avoid leaks in long-running Flask procs.
"""

import gc
import logging
import os
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
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
            Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
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
