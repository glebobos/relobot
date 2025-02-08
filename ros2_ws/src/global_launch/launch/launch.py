from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the diffbot launch file
    diffbot_launch_file = os.path.join(
        get_package_share_directory('diff_drive_hardware'),
        'launch',
        'diffbot.launch.py'
    )

    return LaunchDescription([
        # Include the diffbot launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(diffbot_launch_file)
        ),

        # Add the web server node
        Node(
            package='web_server',
            executable='web_server_node',
            name='web_server_node',
            output='screen',
            parameters=[{
                # Add any parameters if needed
            }]
        ),
    ])