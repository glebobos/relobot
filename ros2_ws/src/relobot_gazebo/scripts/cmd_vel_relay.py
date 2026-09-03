#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    """Relays /cmd_vel to /diff_drive_controller/cmd_vel_unstamped for Gazebo diff_drive_controller."""

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self._pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self._sub = self.create_subscription(Twist, '/cmd_vel', self._callback, 10)
        self.get_logger().info('cmd_vel relay initialized (/cmd_vel -> /diff_drive_controller/cmd_vel_unstamped)')

    def _callback(self, msg: Twist):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
