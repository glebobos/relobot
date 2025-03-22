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
                    'min_height': -0.1,
                    'max_height': 0.3,
                    'angle_min': -0.60,
                    'angle_max': 0.39,
                    'range_max': 4.0,
                    # 'angle_increment': 0.001,
                }
            ],
            remappings=[('/cloud_in', '/cloud_in')]
        )
    ])
