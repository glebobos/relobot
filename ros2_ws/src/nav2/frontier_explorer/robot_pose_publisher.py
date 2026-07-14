from __future__ import annotations

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
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
        self.declare_parameter('max_tf_age_sec', 1.0)

        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._max_tf_age_sec = self.get_parameter('max_tf_age_sec').get_parameter_value().double_value

        # High-rate pose stream: stale samples are useless, so keep only the
        # latest one. Reliability is left at RELIABLE (the QoSProfile default)
        # rather than BEST_EFFORT: a BEST_EFFORT publisher is incompatible
        # with the RELIABLE subscribers used elsewhere (rosbridge_rust's
        # default r2r QosProfile, `ros2 topic echo`, etc.) — DDS won't even
        # connect them, silently dropping every message rather than just the
        # occasional one.
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pose_pub = self.create_publisher(PoseStamped, publish_topic, pose_qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(1.0 / publish_rate_hz, self._tick)

    def _tick(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Could not look up transform from {self._map_frame} to '
                f'{self._base_frame}: {e}',
                throttle_duration_sec=1.0,
            )
            return

        age_sec = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        if age_sec > self._max_tf_age_sec:
            self.get_logger().warn(
                f'Transform from {self._map_frame} to {self._base_frame} is '
                f'{age_sec:.2f}s old (> {self._max_tf_age_sec}s threshold); skipping publish.',
                throttle_duration_sec=1.0,
            )
            return

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
