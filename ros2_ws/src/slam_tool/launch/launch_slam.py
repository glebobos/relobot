#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # SLAM Toolbox configuration
    slam_config_file = os.path.join(
        get_package_share_directory('slam_tool'),
        'config',
        'slam_toolbox_config.yaml'
    )

    # EKF configuration for robot_localization (update the package and path as needed)
    ekf_config_file = os.path.join(
        get_package_share_directory('slam_tool'),
        'config',
        'ekf.yaml'
    )

    # Nav2 configuration
    nav2_config_file = os.path.join(
        get_package_share_directory('slam_tool'),
        'config',
        'nav2_params.yaml'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # or the synchronous node if preferred
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    # Nav2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': False},
                   {'autostart': True},
                   {'node_names': ['controller_server',
                                 'smoother_server',
                                 'planner_server',
                                 'behavior_server',
                                 'bt_navigator',
                                 'waypoint_follower',
                                 'velocity_smoother']}]
    )

    # Nav2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_config_file],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config_file]
    )

    # Nav2 Smoother Server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_config_file]
    )

    # Nav2 Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config_file]
    )

    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config_file]
    )

    # Nav2 Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config_file]
    )

    # Nav2 Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config_file],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
                   ('/cmd_vel_smoothed', '/cmd_vel')]
    )

    return LaunchDescription([
        slam_toolbox_node,
        ekf_node,
        lifecycle_manager,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother
    ])
