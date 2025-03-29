#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the config file from the slam_tool package
    config_file = os.path.join(
        get_package_share_directory('slam_tool'),
        'config',
        'slam_toolbox_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',  # or synchronous node if preferred
            name='slam_toolbox',
            output='screen',
            parameters=[config_file]
        )
    ])
