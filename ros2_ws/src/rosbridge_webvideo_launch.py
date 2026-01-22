#!/usr/bin/env python3
"""
Launch file to start both rosbridge_server and web_video_server together.
This provides WebSocket access to ROS topics via rosbridge (port 9090)
and efficient HTTP video streaming via web_video_server (port 8080).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Rosbridge WebSocket server (port 9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
            }]
        ),
        
        # Web Video Server for HTTP streaming (port 8080)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': 8080,
                'server_threads': 1,
                'ros_threads': 2,
                'default_stream_type': 'mjpeg',
            }]
        ),
    ])
