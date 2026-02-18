#!/usr/bin/env python3
"""
Bridge: looks up AprilTag TF and publishes as PoseStamped
on /detected_dock_pose for opennav_docking.
Caches last known pose so docking continues when tag leaves FOV.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros


class DockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')
        self.declare_parameter('dock_tag_frame', 'tag25h9:0')
        self.declare_parameter('base_frame', 'odom')

        self.tag_frame = self.get_parameter('dock_tag_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.timer = self.create_timer(0.1, self._tick)
        self.last_pose = None

        self.get_logger().info(
            f'Dock pose publisher: {self.base_frame} -> {self.tag_frame}')

    def _tick(self):
        now = self.get_clock().now().to_msg()
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.tag_frame, rclpy.time.Time())
            msg = PoseStamped()
            msg.header.frame_id = t.header.frame_id
            msg.header.stamp = now  # Always use current time
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
            self.last_pose = msg
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        # Always publish last known pose with current timestamp
        if self.last_pose is not None:
            self.last_pose.header.stamp = now
            self.pub.publish(self.last_pose)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DockPosePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
