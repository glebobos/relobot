#!/usr/bin/env python3
# Copyright 2026 ReloBot Contributors
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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class KnivesMock(Node):
    """
    Mock node for ReloBot mower knives RPM in Gazebo simulation.
    Subscribes to /knives/set_rpm and echoes the requested value back
    to /knives/current_rpm at 10 Hz.
    """

    def __init__(self):
        super().__init__('knives_mock')
        self.current_rpm = 0.0

        self.sub = self.create_subscription(
            Float32,
            '/knives/set_rpm',
            self.set_rpm_callback,
            10
        )
        self.pub = self.create_publisher(
            Float32,
            '/knives/current_rpm',
            10
        )
        # 10 Hz periodic telemetry publication
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Knives simulation mock node initialized.')

    def set_rpm_callback(self, msg: Float32):
        new_rpm = float(msg.data)
        if abs(new_rpm - self.current_rpm) > 1e-3:
            self.get_logger().debug(f'Set RPM changed: {self.current_rpm} -> {new_rpm}')
            self.current_rpm = new_rpm

    def timer_callback(self):
        msg = Float32()
        msg.data = self.current_rpm
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KnivesMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
