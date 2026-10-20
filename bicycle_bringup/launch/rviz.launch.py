# Copyright 2024-2026 Ashraf AlAssi, Mhd. Osama Keakaty, Sarah AlAhmad
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

"""
Start RViz with the bicycle's SLAM or navigation view.

Included by sim_bringup.launch.py; on the real robot run it on a development machine.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('bicycle_bringup'), 'rviz', [LaunchConfiguration('config'), '.rviz']]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config',
                default_value='slam',
                choices=['slam', 'navigation'],
                description='RViz view: SLAM (map, scan) or navigation (plus costmaps, plan).',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the simulation (/clock) time source.',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            ),
        ]
    )
