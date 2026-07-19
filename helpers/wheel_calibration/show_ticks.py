#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
import math

class ShowTicksNode(Node):
    def __init__(self):
        super().__init__('show_ticks_node')
        self.sub = self.create_subscription(
            JointState,
            '/robot_joint_states',
            self.callback,
            qos_profile_sensor_data
        )
        self.counts_per_rev = 60.0
        self.rad_to_ticks = self.counts_per_rev / (2.0 * math.pi)
        self.get_logger().info("Show Ticks Node initialized. Waiting for /robot_joint_states messages...")
        print("\n" + "="*112)
        print(f"{'Time (s)':<10} | {'Left Ticks':<12} | {'Left Pos (rad)':<15} | {'Left Vel (rad/s)':<17} || {'Right Ticks':<12} | {'Right Pos (rad)':<15} | {'Right Vel (rad/s)':<17}")
        print("="*112)

    def callback(self, msg):
        left_pos, right_pos = 0.0, 0.0
        left_vel, right_vel = 0.0, 0.0
        
        got_left = False
        got_right = False
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':
                left_pos = msg.position[i]
                left_vel = msg.velocity[i]
                got_left = True
            elif name == 'right_wheel_joint':
                right_pos = msg.position[i]
                right_vel = msg.velocity[i]
                got_right = True
                
        if not got_left or not got_right:
            if len(msg.position) >= 2:
                left_pos = msg.position[0]
                left_vel = msg.velocity[0]
                right_pos = msg.position[1]
                right_vel = msg.velocity[1]
                
        left_ticks = int(round(left_pos * self.rad_to_ticks))
        right_ticks = int(round(right_pos * self.rad_to_ticks))
        
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        print(f"{stamp:<10.3f} | {left_ticks:<12} | {left_pos:<15.4f} | {left_vel:<17.4f} || {right_ticks:<12} | {right_pos:<15.4f} | {right_vel:<17.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = ShowTicksNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Show Ticks Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
