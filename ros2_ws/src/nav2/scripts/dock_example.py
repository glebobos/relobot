#!/usr/bin/env python3
"""
Example script to test docking functionality with Nav2 Docking Server.
This demonstrates how to dock and undock the robot at the configured dock location.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot, UndockRobot
from geometry_msgs.msg import PoseStamped
import time


class DockingTester(Node):
    def __init__(self):
        super().__init__('docking_tester')
        
        # Create action clients for docking and undocking
        self.docking_client = ActionClient(self, DockRobot, 'dock_robot')
        self.undocking_client = ActionClient(self, UndockRobot, 'undock_robot')
        
        self.get_logger().info('Docking Tester Node Initialized')

    def dock_robot(self, dock_id='home_dock', navigate_to_staging=True):
        """
        Send a DockRobot action request.
        
        Args:
            dock_id: The ID of the dock to use (default: 'home_dock')
            navigate_to_staging: Whether to navigate to staging pose first
        """
        self.get_logger().info('Waiting for DockRobot action server...')
        if not self.docking_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DockRobot action server not available!')
            return False

        # Create goal message
        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.navigate_to_staging_pose = navigate_to_staging
        goal_msg.max_staging_time = 60.0  # 60 seconds max to reach staging pose

        self.get_logger().info(f'Sending docking request to dock: {dock_id}')
        self.get_logger().info(f'Navigate to staging: {navigate_to_staging}')
        
        # Send goal
        send_goal_future = self.docking_client.send_goal_async(
            goal_msg,
            feedback_callback=self._dock_feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Docking request was rejected!')
            return False

        self.get_logger().info('Docking request accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'Docking successful! Retries: {result.num_retries}')
            return True
        else:
            self.get_logger().error(f'Docking failed! Error code: {result.error_code}')
            return False

    def undock_robot(self):
        """Send an UndockRobot action request."""
        self.get_logger().info('Waiting for UndockRobot action server...')
        if not self.undocking_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('UndockRobot action server not available!')
            return False

        # Create goal message
        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = 'manual_dock'
        goal_msg.max_undocking_time = 30.0  # 30 seconds max

        self.get_logger().info('Sending undocking request...')
        
        # Send goal
        send_goal_future = self.undocking_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Undocking request was rejected!')
            return False

        self.get_logger().info('Undocking request accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info('Undocking successful!')
            return True
        else:
            self.get_logger().error(f'Undocking failed! Error code: {result.error_code}')
            return False

    def _dock_feedback_callback(self, feedback_msg):
        """Callback for docking feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Docking feedback - State: {feedback.state}, '
            f'Time: {feedback.docking_time.sec}s, '
            f'Retries: {feedback.num_retries}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    tester = DockingTester()
    
    try:
        print("\n" + "="*60)
        print("Nav2 Docking Server - Test Script")
        print("="*60)
        print("\nThis script will test docking at 'home_dock' (0,0,0 in map)")
        print("\nOptions:")
        print("  1. Dock (with navigation to staging pose)")
        print("  2. Dock (skip navigation, assume already near)")
        print("  3. Undock")
        print("  4. Test cycle (dock -> wait -> undock)")
        print("  5. Exit")
        print("="*60)
        
        while True:
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == '1':
                tester.dock_robot(dock_id='home_dock', navigate_to_staging=True)
            
            elif choice == '2':
                tester.dock_robot(dock_id='home_dock', navigate_to_staging=False)
            
            elif choice == '3':
                tester.undock_robot()
            
            elif choice == '4':
                print("\nStarting test cycle...")
                if tester.dock_robot(dock_id='home_dock', navigate_to_staging=True):
                    print("Waiting 5 seconds...")
                    time.sleep(5)
                    tester.undock_robot()
                else:
                    print("Docking failed, skipping undocking")
            
            elif choice == '5':
                print("Exiting...")
                break
            
            else:
                print("Invalid choice, please enter 1-5")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
