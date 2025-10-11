from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Nav2 bringup
    nav2_bringup_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    # Exploration node
    exploration_launch_file = os.path.join(
        get_package_share_directory('nav2_exploration'), 'launch', 'exploration_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_file),
            launch_arguments={'use_sim_time': 'False'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_launch_file),
            launch_arguments={'use_sim_time': 'False'}.items(),
        ),
    ])