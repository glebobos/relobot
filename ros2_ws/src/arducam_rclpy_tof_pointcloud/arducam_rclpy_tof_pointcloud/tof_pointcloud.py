from argparse import ArgumentParser
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
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

MAX_DISTANCE=4000

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

        # Создаём Publisher
        self.publisher_pcl_ = self.create_publisher(
            PointCloud2, "cloud_in", 10
        )

        self.fx = self.tof_.getControl(Control.INTRINSIC_FX) / 100
        self.fy = self.tof_.getControl(Control.INTRINSIC_FY) / 100

        self.width_ = self.frame_width
        self.height_ = self.frame_height

        self.u_grid, self.v_grid = np.meshgrid(
            np.arange(self.width_), np.arange(self.height_)
        )

        self.timer_ = self.create_timer(1.0 / 30.0, self.update)

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
            tof.setControl(Control.RANGE, 4000)
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

    def update(self):
        frame = self.tof_.requestFrame(20)
        if frame is None or not isinstance(frame, DepthData):
            if frame:
                self.tof_.releaseFrame(frame)
            return

        depth_buf = frame.depth_data
        confidence_buf = frame.confidence_data

        depth_buf[confidence_buf < 30] = 0
        z = depth_buf.astype(np.float32) / 1000.0

        invalid_mask = (z <= 0.0)
        z[invalid_mask] = np.nan

        x = (self.u_grid - (self.width_ / 2.0)) * z / self.fx
        y = (self.v_grid - (self.height_ / 2.0)) * z / self.fy

        points = np.dstack((z, -x, -y)).reshape(-1, 3)
        valid_mask = ~np.isnan(points).any(axis=1)
        points = points[valid_mask]

        self.header.stamp = self.get_clock().now().to_msg()

        pc2_msg = point_cloud2.create_cloud_xyz32(self.header, points)
        self.publisher_pcl_.publish(pc2_msg)

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
