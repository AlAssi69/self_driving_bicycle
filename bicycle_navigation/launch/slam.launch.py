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
Run slam_toolbox in mapping or localization mode; it provides /map and map->odom.

mapping:      online async SLAM, starts a new map.
localization: localizes against the serialized pose graph given by `map`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = FindPackageShare('bicycle_navigation')
    slam_launch_dir = PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = EqualsSubstitution(LaunchConfiguration('mode'), 'localization')

    params_file = PathJoinSubstitution([share, 'config', 'slam.yaml'])
    localization_params_file = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'map_file_name': LaunchConfiguration('map')},
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the simulation (/clock) time source.',
            ),
            DeclareLaunchArgument(
                'mode',
                default_value='mapping',
                choices=['mapping', 'localization'],
                description='Build a new map, or localize in the map given by `map`.',
            ),
            DeclareLaunchArgument(
                'map',
                default_value=PathJoinSubstitution([share, 'maps', 'obstacles_course']),
                description='Serialized slam_toolbox map, without extension (localization only).',
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([slam_launch_dir, 'online_async_launch.py']),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': params_file,
                }.items(),
                condition=UnlessCondition(localization),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([slam_launch_dir, 'localization_launch.py']),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': localization_params_file,
                }.items(),
                condition=IfCondition(localization),
            ),
        ]
    )
