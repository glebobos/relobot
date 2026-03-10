"""
Launch file to start the rosbridge_server.
This provides WebSocket access to ROS topics via rosbridge (port 9090).
Note: web_video_server was moved to camera_with_apriltag.launch.py for zero-copy.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'default_call_service_timeout': 5.0,
                'call_services_in_new_thread': True,
                'send_action_goals_in_new_thread': True,
            }]
        ),
    ])
