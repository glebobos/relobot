#!/usr/bin/env python3
# Copyright 2026 ReloBot Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_relobot_gazebo = get_package_share_directory('relobot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch configurations
    world_arg = LaunchConfiguration('world')
    gui_arg = LaunchConfiguration('gui')
    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    x_pose_arg = LaunchConfiguration('x_pose')
    y_pose_arg = LaunchConfiguration('y_pose')
    z_pose_arg = LaunchConfiguration('z_pose')
    yaw_arg = LaunchConfiguration('yaw')
    web_video_arg = LaunchConfiguration('web_video')

    # Declared arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='garden_world.sdf',
        description='World file name (e.g. garden_world.sdf, obstacle_world.sdf, empty_world.sdf)'
    )

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI if true, headless server if false'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_x_pose_cmd = DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial robot X pose')
    declare_y_pose_cmd = DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial robot Y pose')
    declare_z_pose_cmd = DeclareLaunchArgument('z_pose', default_value='0.15', description='Initial robot Z pose')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot Yaw')

    declare_web_video_cmd = DeclareLaunchArgument(
        'web_video',
        default_value='true',
        description='Launch web_video_server for streaming camera to web UI'
    )

    # Set GZ resource path for models/worlds
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_relobot_gazebo, 'worlds'), ':',
            os.path.join(pkg_relobot_gazebo, 'models'), ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    # World path substitution
    world_path = PathJoinSubstitution([
        pkg_relobot_gazebo,
        'worlds',
        world_arg
    ])

    # Gazebo Sim Launch
    # Uses -r (run physics immediately) and conditionally -s (headless server only if gui is false)
    gz_args = PythonExpression([
        "'-r ' + ('' if '", gui_arg, "'.lower() == 'true' else '-s ') + '", world_path, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Robot Description from Xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([pkg_relobot_gazebo, 'config', 'diffbot_sim.urdf.xacro']),
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher (must use sim_time in simulation to match TF timestamps)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time_arg}]
    )

    # Spawn Robot Entity in Gazebo (pass URDF string directly to avoid waiting on latched topic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'diffbot',
            '-string', robot_description_content,
            '-x', x_pose_arg,
            '-y', y_pose_arg,
            '-z', z_pose_arg,
            '-Y', yaw_arg,
        ]
    )

    # ROS-Gazebo Parameter Bridge
    bridge_config_file = os.path.join(pkg_relobot_gazebo, 'config', 'bridge_config.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_file,
            'use_sim_time': use_sim_time_arg,
        }]
    )

    # Controllers Spawner Watchdog (robustly waits for /controller_manager without dying prematurely)
    controller_spawner = Node(
        package='relobot_gazebo',
        executable='controller_spawner.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_arg}],
    )

    # Delay controller spawning until after robot entity is spawned in Gazebo
    delay_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[controller_spawner],
        )
    )

    # EKF Localization Node
    ekf_config_file = os.path.join(pkg_relobot_gazebo, 'config', 'ekf_sim.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time_arg}]
    )

    # Web Video Server for streaming camera feed to web interface
    web_video_server_node = Node(
        condition=IfCondition(web_video_arg),
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0',
            'server_threads': 2,
            'ros_threads': 2,
            'type': 'mjpeg',
        }]
    )

    # Command velocity relay (/cmd_vel -> /diff_drive_controller/cmd_vel_unstamped)
    cmd_vel_relay_node = Node(
        package='relobot_gazebo',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_arg}],
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_gui_cmd,
        declare_use_sim_time_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_z_pose_cmd,
        declare_yaw_cmd,
        declare_web_video_cmd,
        gz_resource_path,
        gz_sim,
        robot_state_publisher_node,
        spawn_robot,
        ros_gz_bridge,
        delay_controller_spawner,
        cmd_vel_relay_node,
        ekf_node,
        web_video_server_node,
    ])
