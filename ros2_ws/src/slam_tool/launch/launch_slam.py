#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from lifecycle_msgs.msg import Transition

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

    # Check for existing serialized map
    map_serialized_path = '/ros2_ws/map_serialized'
    map_exists = os.path.isfile(map_serialized_path + '.posegraph') or os.path.isfile(map_serialized_path + '.data')

    print(f"DEBUG: Checking for map at {map_serialized_path}")
    print(f"DEBUG: Map exists: {map_exists}")

    slam_params = {
        'mode': 'mapping',  # if map_exists else 'mapping', TODO optimize localisation mode
        'map_file_name': map_serialized_path if map_exists else '',
    }

    # Use LifecycleNode for slam_toolbox (required in Jazzy)
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_config_file, slam_params]
    )

    # Auto-configure slam_toolbox after it starts
    configure_slam_toolbox = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Auto-activate slam_toolbox after configuration
    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    return LaunchDescription([
        slam_toolbox_node,
        configure_slam_toolbox,
        activate_slam_toolbox,
        ekf_node
    ])
