#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # SLAM Toolbox configuration
    slam_config_file = os.path.join(
        get_package_share_directory('slam_tool'),
        'config',
        'slam_toolbox_config.yaml'
    )

    # EKF configuration for robot_localization (update the package and path as needed)
    ekf_config_file = os.path.join(
        get_package_share_directory('slam_tool'),
        'config',
        'ekf.yaml'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # or the synchronous node if preferred
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    return LaunchDescription([
        slam_toolbox_node,
        ekf_node
    ])
