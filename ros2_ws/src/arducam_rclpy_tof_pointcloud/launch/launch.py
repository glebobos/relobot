from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your filter chain configuration file
    config_file = os.path.join(
        get_package_share_directory('arducam_rclpy_tof_pointcloud'),
        'config',
        'scan_filter_chain.yaml'
    )

    return LaunchDescription([
        # TOF pointcloud node
        Node(
            package='arducam_rclpy_tof_pointcloud',
            executable='tof_pointcloud',
            name='tof_pointcloud',
            output='screen'
        ),
        # Convert pointcloud to raw laserscan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'min_height': -0.1,
                'max_height': 0.0,
                'angle_min': -0.61,
                'angle_max': 0.61,
                'range_max': 4.0,
                'angle_increment': 0.0087,
            }],
            remappings=[('/cloud_in', '/cloud_in')]
        ),
        # Apply laser scan filters using the filter chain from laser_filters
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            output='screen',
            parameters=[config_file],
        )
    ])
