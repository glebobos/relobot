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
        # 1. Wait for controller_manager service to appear
        attempt = 0
        while not self.cli.wait_for_service(timeout_sec=1.0):
            attempt += 1
            if attempt % 5 == 0:
                self.get_logger().info('Waiting for /controller_manager/list_controllers service to become available...')

        self.get_logger().info('/controller_manager is available! Spawning controllers sequentially...')

        # 2. Spawn joint_state_broadcaster first, then diff_drive_controller sequentially
        controllers = ['joint_state_broadcaster', 'diff_drive_controller']
        max_retries = 5

        for controller in controllers:
            spawned = False
            for retry in range(1, max_retries + 1):
                cmd = [
                    'ros2', 'run', 'controller_manager', 'spawner',
                    controller,
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '5',
                    '--ros-args', '-p', 'use_sim_time:=true'
                ]
                self.get_logger().info(f'Spawning {controller} (attempt {retry}/{max_retries})...')
                res = subprocess.run(cmd)

                if res.returncode == 0:
                    self.get_logger().info(f'Successfully activated {controller}!')
                    spawned = True
                    break

                self.get_logger().warn(f'Spawner for {controller} returned code {res.returncode}. Retrying in 1s...')
                time.sleep(1.0)

            if not spawned:
                self.get_logger().error(f'Failed to spawn {controller} after {max_retries} attempts.')
                return 1

        self.get_logger().info('All controllers successfully spawned and active!')
        return 0


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
