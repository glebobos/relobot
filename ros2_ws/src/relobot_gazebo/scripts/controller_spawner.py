#!/usr/bin/env python3
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers


class ControllerSpawnerWatchdog(Node):
    def __init__(self):
        super().__init__('controller_spawner_watchdog')
        self.cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.get_logger().info('Controller Spawner Watchdog started. Waiting for /controller_manager...')

    def wait_and_spawn(self):
        # 1. Wait indefinitely for controller_manager service to appear
        attempt = 0
        while not self.cli.wait_for_service(timeout_sec=2.0):
            attempt += 1
            if attempt % 3 == 0:
                self.get_logger().info('Waiting for /controller_manager/list_controllers service to become available...')

        self.get_logger().info('/controller_manager is available! Spawning controllers...')

        # 2. Spawn joint_state_broadcaster and diff_drive_controller with retry loop
        controllers = ['joint_state_broadcaster', 'diff_drive_controller']
        max_retries = 10
        
        for retry in range(1, max_retries + 1):
            cmd = [
                'ros2', 'run', 'controller_manager', 'spawner',
                *controllers,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '30',
                '--ros-args', '-p', 'use_sim_time:=true'
            ]
            self.get_logger().info(f'Running: {" ".join(cmd)} (attempt {retry}/{max_retries})')
            res = subprocess.run(cmd)
            
            if res.returncode == 0:
                self.get_logger().info('Successfully spawned joint_state_broadcaster and diff_drive_controller!')
                return 0
            
            self.get_logger().warn(f'Spawner returned code {res.returncode}. Retrying in 2 seconds...')
            time.sleep(2.0)

        self.get_logger().error(f'Failed to spawn controllers after {max_retries} attempts.')
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSpawnerWatchdog()
    try:
        ret = node.wait_and_spawn()
        return ret
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
