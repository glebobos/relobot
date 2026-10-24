# Copyright 2025 ReloBot Contributors
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diff_drive_hardware"), "config", "diffbot.urdf.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = os.path.join(
        get_package_share_directory("diff_drive_hardware"),
        "config",
        "diffbot_controllers.yaml",
    )

    # EKF configuration for robot_localization
    ekf_config_file = os.path.join(
        get_package_share_directory('diff_drive_hardware'),
        'config',
        'ekf.yaml'
    )

    # Robot state publisher configuration
    robot_state_pub_config = os.path.join(
        get_package_share_directory('diff_drive_hardware'),
        'config',
        'robot_state_publisher.yaml'
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value=EnvironmentVariable("ROS_LOG_LEVEL", default_value="WARN"),
        description="ROS 2 logger severity level",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        on_exit=Shutdown(),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, robot_state_pub_config],
        arguments=["--ros-args", "--log-level", log_level],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--param-file", robot_controllers,
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--param-file", robot_controllers,
            "--controller-ros-args", "--ros-args --remap ~/cmd_vel:=/cmd_vel",
        ],
    )

    init_cmd_vel_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/TwistStamped',
             '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'],
        output='log',
    )

    delay_init_cmd_vel = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[init_cmd_vel_cmd],
        )
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    nodes = [
        declare_log_level_cmd,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        delay_init_cmd_vel,
        ekf_node,
    ]

    return LaunchDescription(nodes)
