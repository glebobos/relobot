from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the core navigation launch file
    nav_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    # The exploration node itself
    explorer_node = Node(
        package='nav2_exploration',
        executable='explorer',
        name='explorer',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        # Launch the core Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_file),
            launch_arguments={'use_sim_time': 'False'}.items(),
        ),
        # Launch the explorer node
        explorer_node
    ])