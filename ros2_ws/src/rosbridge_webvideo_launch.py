"""
Launch file to start the rosbridge_server.
This provides WebSocket access to ROS topics via rosbridge (port 9090).
Note: web_video_server was moved to camera.launch.py for zero-copy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = LaunchConfiguration('log_level')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value=EnvironmentVariable('ROS_LOG_LEVEL', default_value='WARN'),
        description='ROS 2 logger severity level',
    )

    return LaunchDescription([
        declare_log_level_cmd,
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'port': 9091,
                'default_call_service_timeout': 5.0,
                'call_services_in_new_thread': True,
                'send_action_goals_in_new_thread': True,
            }]
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
