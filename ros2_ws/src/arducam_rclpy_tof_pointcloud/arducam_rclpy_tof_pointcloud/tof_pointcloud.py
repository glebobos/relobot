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
from typing import Optional, Tuple, Union

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# Image processing imports
import numpy as np
import cv2
from cv_bridge import CvBridge

# ArduCam imports
from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

# Constants
MAX_DISTANCE = 4000
CONFIDENCE_THRESHOLD = 50
FRAME_RATE = 20.0  # Hz

class Option:
    """Configuration options for camera setup"""
    cfg: Optional[str] = None

class TOFPublisher(Node):
    """
    ROS2 node for publishing ToF (Time of Flight) sensor data
    
    Publishes:
    - Point cloud
    - Depth image
    - Confidence image
    """
    def __init__(self, options: Option):
        super().__init__("arducam_tof_pointcloud")

        # Initialize camera
        self.tof_ = self._init_camera(options)
        if self.tof_ is None:
            raise RuntimeError("Failed to initialize ArduCam ToF camera")

        # Configure message headers
        self.header = Header()
        self.header.frame_id = "sensor_frame"
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()

        # Create publishers
        self.publisher_pcl_ = self.create_publisher(
            PointCloud2, "cloud_in", 10
        )
        self.publisher_depth_image_ = self.create_publisher(
            Image, "depth_image", 10
        )
        self.publisher_confidence_image_ = self.create_publisher(
            Image, "confidence_image", 10
        )

        # Get camera intrinsic parameters
        self.fx = self.tof_.getControl(Control.INTRINSIC_FX) / 100
        self.fy = self.tof_.getControl(Control.INTRINSIC_FY) / 100
        self.width_ = self.frame_width
        self.height_ = self.frame_height
        self.camera_range = self.tof_.getControl(Control.RANGE)
        
        # Create coordinate grids for point cloud generation
        self.u_grid, self.v_grid = np.meshgrid(
            np.arange(self.width_), np.arange(self.height_)
        )
        
        # Start the update timer
        self.timer_ = self.create_timer(1.0 / FRAME_RATE, self.update)

    def _init_camera(self, options: Option) -> Optional[ArducamCamera]:
        """
        Initialize the ArduCam ToF camera
        
        Args:
            options: Camera configuration options
            
        Returns:
            ArducamCamera object if successful, None otherwise
        """
        self.get_logger().info("Initializing ArduCam ToF camera...")
        tof = ArducamCamera()
        
        # Open camera with config file or default CSI connection
        if options.cfg is not None:
            ret = tof.openWithFile(options.cfg, 0)
            connection_type = f"config file: {options.cfg}"
        else:
            ret = tof.open(Connection.CSI, 0)
            connection_type = "CSI connection"
            
        if ret != 0:
            self.get_logger().error(f"Failed to open camera using {connection_type}. Error code: {ret}")
            return None

        # Start camera in depth mode
        ret = tof.start(FrameType.DEPTH)
        if ret != 0:
            self.get_logger().error(f"Failed to start camera. Error code: {ret}")
            tof.close()
            return None

        # Get camera information and configure frame dimensions
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

        self.get_logger().info(f"Camera initialized successfully: {width}x{height}, device type: {info.device_type}")
        return tof

    def _apply_confidence_filter(self, image: np.ndarray, confidence: np.ndarray) -> np.ndarray:
        """
        Apply confidence filtering to an image
        
        Args:
            image: Input image
            confidence: Confidence values for each pixel
            
        Returns:
            Filtered image with low confidence areas set to black
        """
        image = np.nan_to_num(image)
        image[confidence < CONFIDENCE_THRESHOLD] = (0, 0, 0)
        return image

    def create_depth_image(self, depth_data: np.ndarray, confidence_data: np.ndarray) -> np.ndarray:
        """
        Create colored depth image with rainbow colormap
        
        Args:
            depth_data: Raw depth data
            confidence_data: Confidence values for each pixel
            
        Returns:
            Colorized depth image with confidence filtering
        """
        # Normalize depth to 0-255 range
        normalized = (depth_data * (255.0 / self.camera_range)).astype(np.uint8)
        
        # Apply rainbow colormap
        colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_RAINBOW)
        
        # Apply confidence filtering
        return self._apply_confidence_filter(colorized, confidence_data)

    def create_confidence_image(self, confidence_data: np.ndarray) -> np.ndarray:
        """
        Create normalized confidence image
        
        Args:
            confidence_data: Raw confidence data
            
        Returns:
            Normalized confidence image (0-255)
        """
        # Convert to float32 for normalization
        confidence_normalized = confidence_data.astype(np.float32)
        
        # Normalize to 0-255 range
        cv2.normalize(confidence_normalized, confidence_normalized, 0, 255, cv2.NORM_MINMAX)
        
        return confidence_normalized.astype(np.uint8)

    def _create_point_cloud(self, depth_data: np.ndarray, confidence_data: np.ndarray) -> np.ndarray:
        """
        Create a 3D point cloud from depth data
        
        Args:
            depth_data: Raw depth data in mm
            confidence_data: Confidence values for each pixel
            
        Returns:
            Nx3 array of 3D points (z, -x, -y) in meters
        """
        # Filter by confidence and convert to meters
        depth_filtered = depth_data.copy()
        depth_filtered[confidence_data < CONFIDENCE_THRESHOLD] = 0
        z = depth_filtered.astype(np.float32) / 1000.0  # mm to meters
        
        # Mark invalid points as NaN
        z[z <= 0.0] = np.nan
        
        # Calculate x and y coordinates using camera intrinsics
        x = (self.u_grid - (self.width_ / 2.0)) * z / self.fx
        y = (self.v_grid - (self.height_ / 2.0)) * z / self.fy
        
        # Stack points and filter out invalid ones
        points = np.dstack((z, -x, -y)).reshape(-1, 3)
        valid_mask = ~np.isnan(points).any(axis=1)
        
        return points[valid_mask]
    
    def update(self) -> None:
        """
        Timer callback to process frame data and publish messages
        """
        # Request frame from camera
        frame = self.tof_.requestFrame(int(1000/FRAME_RATE))
        if frame is None or not isinstance(frame, DepthData):
            if frame:
                self.tof_.releaseFrame(frame)
            return

        try:
            # Extract frame data
            depth_data = frame.depth_data
            confidence_data = frame.confidence_data
            
            # Update timestamp for all messages
            self.header.stamp = self.get_clock().now().to_msg()
            
            # Process and publish point cloud
            points = self._create_point_cloud(depth_data, confidence_data)
            pc2_msg = point_cloud2.create_cloud_xyz32(self.header, points)
            self.publisher_pcl_.publish(pc2_msg)
            
            # Process and publish depth image
            depth_image = self.create_depth_image(depth_data, confidence_data)
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, "bgr8")
            depth_image_msg.header = self.header
            self.publisher_depth_image_.publish(depth_image_msg)
            
            # Process and publish confidence image
            confidence_image = self.create_confidence_image(confidence_data)
            confidence_image_msg = self.bridge.cv2_to_imgmsg(confidence_image, "mono8")
            confidence_image_msg.header = self.header
            self.publisher_confidence_image_.publish(confidence_image_msg)
            
        finally:
            # Always release the frame
            self.tof_.releaseFrame(frame)

def main(args=None):
    """
    Main function to initialize and run the ToF point cloud publisher
    
    Args:
        args: Command line arguments
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = ArgumentParser(description="ArduCam ToF pointcloud publisher for ROS2")
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")
    ns, _ = parser.parse_known_args()
    
    # Create camera options
    options = Option()
    options.cfg = ns.cfg
    
    try:
        # Create and initialize the ToF publisher node
        tof_publisher = TOFPublisher(options)
        
        # Run with a multi-threaded executor
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(tof_publisher)
        
        # Start processing
        print("ArduCam ToF pointcloud publisher running. Press Ctrl+C to exit.")
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Clean up resources
        executor = MultiThreadedExecutor()
        executor.shutdown()
        if 'tof_publisher' in locals():
            tof_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()