from __future__ import annotations

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RobotPosePublisher(Node):
    """Republishes the robot's map-frame pose as a single PoseStamped topic.

    Looks up the `map` -> `base_link` transform via tf2 and republishes it,
    so downstream consumers (e.g. the web UI) don't need to combine
    `map` -> `odom` (from SLAM) and `odom` -> `base_link` (from the EKF)
    themselves.
    """

    def __init__(self) -> None:
        super().__init__('robot_pose_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_topic', '/robot_pose')
        self.declare_parameter('publish_rate_hz', 30.0)

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('publish_topic').value, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._tf_ever_available = False

        self.create_timer(1.0 / publish_rate_hz, self._tick)

    def _tick(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
            )
        except Exception as e:
            if self._tf_ever_available:
                self.get_logger().warn(
                    f'Could not look up transform from {self._map_frame} to '
                    f'{self._base_frame}: {e}'
                )
            return

        self._tf_ever_available = True

        pose = PoseStamped()
        pose.header.stamp = t.header.stamp
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation

        self._pose_pub.publish(pose)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
