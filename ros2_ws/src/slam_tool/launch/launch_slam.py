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



    # Check for existing serialized map
    map_serialized_path = '/ros2_ws/map_serialized'
    map_exists = os.path.isfile(map_serialized_path + '.posegraph') or os.path.isfile(map_serialized_path + '.data')

    print(f"DEBUG: Checking for map at {map_serialized_path}")
    print(f"DEBUG: Map exists: {map_exists}")

    slam_params = {
        'mode': 'mapping', # if map_exists else 'mapping', TODO optimize localisation mode
        'map_file_name': map_serialized_path if map_exists else '',
    }
    
    # If we are verifying this iteratively, we might want to ensure we merge with config file
    # But Node parameters lists are merged.
    # However, to override values in the yaml, we should pass them as a dict in parameters list

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file, slam_params]
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
