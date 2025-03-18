from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arducam_rclpy_tof_pointcloud',
            executable='tof_pointcloud',
            name='tof_pointcloud',
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                {
                    'angle_min': -3.1415927410125732,
                    'angle_max': 3.1415927410125732,
                    'range_min': 0.0
                }
            ],
            remappings=[('/cloud_in', '/cloud_in')]
        )
    ])
