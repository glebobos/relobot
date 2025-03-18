from argparse import ArgumentParser
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

class Option:
    cfg: Optional[str]

class TOFPublisher(Node):
    def __init__(self, options: Option):
        super().__init__("arducam")

        self.tof_ = self.__init_camera(options)
        if self.tof_ is None:
            raise Exception("Failed to initialize camera")

        self.depth_msg_ = Float32MultiArray()
        self.header = Header()
        self.header.frame_id = "sensor_frame"

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.publisher_pcl_ = self.create_publisher(
            PointCloud2, "cloud_in", 10
        )
        self.publisher_depth_ = self.create_publisher(
            Float32MultiArray, "depth_frame", 10
        )

        # Get camera intrinsics once (assuming they're not changing frequently)
        self.fx = self.tof_.getControl(Control.INTRINSIC_FX) / 100
        self.fy = self.tof_.getControl(Control.INTRINSIC_FY) / 100

        # Prepare convenient variables
        self.width_ = self.frame_width
        self.height_ = self.frame_height

        # Precompute pixel grid for faster math
        self.u_grid, self.v_grid = np.meshgrid(
            np.arange(self.width_), np.arange(self.height_)
        )

        # Create a timer to read and publish frames at 10 Hz
        self.timer_ = self.create_timer(1.0 / 10.0, self.update)

    def __init_camera(self, options: Option):
        """Initialize camera and start streaming depth data."""
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
            tof.setControl(Control.RANGE, 4)
        elif info.device_type == DeviceType.VGA:
            width = info.width
            height = info.height // 10 - 1
        else:
            # Default fallback if other device types exist
            width = info.width
            height = info.height

        self.frame_width = width
        self.frame_height = height

        print(f"Open camera success, width: {width}, height: {height}")
        return tof

    def update(self):
        """
        Called at the timer rate (e.g., 10 Hz).
        Reads a new frame from the camera, converts to point cloud,
        and publishes both the point cloud and depth message.
        """
        # Grab the latest depth frame
        frame = self.tof_.requestFrame(20)  # 200 ms timeout
        if frame is None:
            # No new frame. Just return.
            return
        if not isinstance(frame, DepthData):
            # Not a depth frame; discard.
            self.tof_.releaseFrame(frame)
            return

        depth_buf = frame.depth_data         # shape: (height, width), uint16
        confidence_buf = frame.confidence_data

        # Filter out low-confidence measurements
        depth_buf[confidence_buf < 30] = 0

        # Convert to float and meters
        z = depth_buf.astype(np.float32) / 1000.0
        # Flatten for Float32MultiArray publishing
        self.depth_msg_.data = z.flatten().tolist()

        # We'll set invalid depths to NaN for point-cloud filtering
        invalid_mask = (z <= 0.0)
        z[invalid_mask] = np.nan

        # Build the (x, y, z) coordinates
        x = (self.u_grid - (self.width_ / 2.0)) * z / self.fx
        y = (self.v_grid - (self.height_ / 2.0)) * z / self.fy

        # Stack into Nx3 and remove any NaNs
        points = np.dstack((x, y, z)).reshape(-1, 3)
        valid_mask = ~np.isnan(points).any(axis=1)
        points = points[valid_mask]

        # Update header time
        self.header.stamp = self.get_clock().now().to_msg()

        # Create and publish the point cloud
        pc2_msg = point_cloud2.create_cloud_xyz32(self.header, points)
        self.publisher_pcl_.publish(pc2_msg)

        # Publish the depth frame
        self.publisher_depth_.publish(self.depth_msg_)

        # Release frame after processing
        self.tof_.releaseFrame(frame)


def main(args=None):
    rclpy.init(args=args)
    parser = ArgumentParser()
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")
    ns = parser.parse_args()

    options = Option()
    options.cfg = ns.cfg

    tof_publisher = TOFPublisher(options)
    try:
        rclpy.spin(tof_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        tof_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
