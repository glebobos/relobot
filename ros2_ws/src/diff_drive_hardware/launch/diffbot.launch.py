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
from launch.actions import Shutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diff_drive_hardware"),
            "config",
            "diffbot_controllers.yaml",
        ]
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

    # Micro-ROS agent — multiserial with persistent hardware symlinks discovery
    import glob
    by_id_devs = sorted(
        glob.glob('/dev/serial/by-id/usb-*Pico*') +
        glob.glob('/dev/serial/by-id/usb-*ReloBot*')
    )
    if by_id_devs:
        serial_devs = by_id_devs
    else:
        # Fallback for environments where by-id symlinks are unavailable (e.g. mock/test)
        serial_devs = sorted(glob.glob('/dev/ttyACM*'))

    # Also include any valid explicit TTY_* overrides from environment
    for k, v in os.environ.items():
        if k.startswith('TTY_') and v and os.path.exists(v):
            if v not in serial_devs:
                serial_devs.append(v)

    if not serial_devs:
        serial_devs = ['/dev/ttyACM0']

    print(f"[DiffBot Launch] Active Micro-ROS serial devices: {serial_devs}")

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='imu_micro_ros_agent',
        arguments=['multiserial', '--devs', ' '.join(serial_devs), '-b', '115200'],
        output='screen',
        on_exit=Shutdown(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("diff_drive_controller/cmd_vel_unstamped", "cmd_vel"),
        ],
        on_exit=Shutdown(),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, robot_state_pub_config],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    nodes = [
        micro_ros_agent,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        ekf_node,
    ]

    return LaunchDescription(nodes)