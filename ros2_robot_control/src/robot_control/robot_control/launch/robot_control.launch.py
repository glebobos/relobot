from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='web_server',
            name='web_server',
            parameters=[{
                'template_path': '/ros2_ws/templates'
            }]
        ),
        Node(
            package='robot_control',
            executable='motor_controller',
            name='motor_controller'
        )
    ])