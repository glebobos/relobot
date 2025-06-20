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

from argparse import ArgumentParser
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
from cv_bridge import CvBridge

from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

MAX_DISTANCE = 4000
CONFIDENCE_THRESHOLD = 30

class Option:
    cfg: Optional[str]

class TOFPublisher(Node):
    def __init__(self, options: Option):
        super().__init__("arducam")

        self.tof_ = self.__init_camera(options)
        if self.tof_ is None:
            raise Exception("Failed to initialize camera")

        self.header = Header()
        self.header.frame_id = "sensor_frame"
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()

        # Create Publishers
        self.publisher_pcl_ = self.create_publisher(
            PointCloud2, "cloud_in", 10
        )
        
        # Add image publishers
        self.publisher_depth_image_ = self.create_publisher(
            Image, "depth_image", 10
        )
        
        self.publisher_confidence_image_ = self.create_publisher(
            Image, "confidence_image", 10
        )

        self.fx = self.tof_.getControl(Control.INTRINSIC_FX) / 100
        self.fy = self.tof_.getControl(Control.INTRINSIC_FY) / 100

        self.width_ = self.frame_width
        self.height_ = self.frame_height

        self.u_grid, self.v_grid = np.meshgrid(
            np.arange(self.width_), np.arange(self.height_)
        )

        # Get camera range for image normalization
        self.camera_range = self.tof_.getControl(Control.RANGE)
        
        self.timer_ = self.create_timer(1.0 / 20.0, self.update)

    def __init_camera(self, options: Option):
        print("Initializing camera...")
        tof = ArducamCamera()
        ret = 0

        if options.cfg is not None:
            ret = tof.openWithFile(options.cfg, 0)
        else:
            ret = tof.open(Connection.CSI, 0)

        if ret != 0:
            print("Failed to open camera. Error code:", ret)
            return None

        ret = tof.start(FrameType.DEPTH)
        if ret != 0:
            print("Failed to start camera. Error code:", ret)
            tof.close()
            return None

        info = tof.getCameraInfo()
        if info.device_type == DeviceType.HQVGA:
            width = info.width
            height = info.height
            tof.setControl(Control.RANGE, MAX_DISTANCE)
        elif info.device_type == DeviceType.VGA:
            width = info.width
            height = info.height // 10 - 1
        else:
            width = info.width
            height = info.height

        self.frame_width = width
        self.frame_height = height

        print(f"Open camera success, width: {width}, height: {height}")
        return tof

    def getPreviewRGB(self, preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
        """Apply confidence filtering to preview image"""
        preview = np.nan_to_num(preview)
        preview[confidence < CONFIDENCE_THRESHOLD] = (0, 0, 0)
        return preview

    def create_depth_image(self, depth_buf: np.ndarray, confidence_buf: np.ndarray) -> np.ndarray:
        """Create colored depth image similar to the example"""
        # Normalize depth to 0-255 range
        result_image = (depth_buf * (255.0 / self.camera_range)).astype(np.uint8)
        
        # Apply rainbow colormap
        result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
        
        # Apply confidence filtering
        result_image = self.getPreviewRGB(result_image, confidence_buf)
        
        return result_image

    def create_confidence_image(self, confidence_buf: np.ndarray) -> np.ndarray:
        """Create normalized confidence image"""
        confidence_normalized = confidence_buf.copy().astype(np.float32)
        cv2.normalize(confidence_normalized, confidence_normalized, 0, 255, cv2.NORM_MINMAX)
        return confidence_normalized.astype(np.uint8)

    def update(self):
        frame = self.tof_.requestFrame(20)
        if frame is None or not isinstance(frame, DepthData):
            if frame:
                self.tof_.releaseFrame(frame)
            return

        depth_buf = frame.depth_data
        confidence_buf = frame.confidence_data

        # Update timestamp
        self.header.stamp = self.get_clock().now().to_msg()

        # Create and publish point cloud
        depth_buf_filtered = depth_buf.copy()
        depth_buf_filtered[confidence_buf < CONFIDENCE_THRESHOLD] = 0
        z = depth_buf_filtered.astype(np.float32) / 1000.0

        invalid_mask = (z <= 0.0)
        z[invalid_mask] = np.nan

        x = (self.u_grid - (self.width_ / 2.0)) * z / self.fx
        y = (self.v_grid - (self.height_ / 2.0)) * z / self.fy

        points = np.dstack((z, -x, -y)).reshape(-1, 3)
        valid_mask = ~np.isnan(points).any(axis=1)
        points = points[valid_mask]

        pc2_msg = point_cloud2.create_cloud_xyz32(self.header, points)
        self.publisher_pcl_.publish(pc2_msg)

        # Create and publish depth image
        depth_image = self.create_depth_image(depth_buf, confidence_buf)
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, "bgr8")
        depth_image_msg.header = self.header
        self.publisher_depth_image_.publish(depth_image_msg)

        # Create and publish confidence image
        confidence_image = self.create_confidence_image(confidence_buf)
        confidence_image_msg = self.bridge.cv2_to_imgmsg(confidence_image, "mono8")
        confidence_image_msg.header = self.header
        self.publisher_confidence_image_.publish(confidence_image_msg)

        self.tof_.releaseFrame(frame)

def main(args=None):
    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")

    ns, unknown = parser.parse_known_args()

    options = Option()
    options.cfg = ns.cfg

    tof_publisher = TOFPublisher(options)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(tof_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        tof_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()