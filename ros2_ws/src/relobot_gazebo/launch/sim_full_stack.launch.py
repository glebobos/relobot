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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_relobot_gazebo = get_package_share_directory('relobot_gazebo')
    pkg_nav2 = get_package_share_directory('nav2')

    world_arg = LaunchConfiguration('world')
    gui_arg = LaunchConfiguration('gui')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='garden_world.sdf',
        description='World file name'
    )

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI if true'
    )

    # 1. Gazebo Simulation + Robot + Bridge + EKF
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_relobot_gazebo, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={
            'world': world_arg,
            'gui': gui_arg,
            'use_sim_time': 'true',
        }.items(),
    )

    # 2. Nav2 + SLAM Toolbox + Explore + Coverage
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_gui_cmd,
        gazebo_sim,
        nav2_bringup,
    ])
