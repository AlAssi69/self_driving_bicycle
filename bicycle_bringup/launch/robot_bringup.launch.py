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
Bring up the real bicycle: description, Arduino and lidar, controllers, and optionally SLAM, Nav2.

Runs on the bicycle's computer, with the same controllers, SLAM and Nav2 as the simulation;
only the hardware layer (bicycle_hardware/hardware.launch.py) replaces Gazebo. View it from a
development machine on the same network with
`ros2 launch bicycle_bringup rviz.launch.py config:=navigation`.
Component arguments (lidar, mode, map, ...) are forwarded automatically;
`ros2 launch bicycle_bringup robot_bringup.launch.py --show-args` lists all of them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def include(package, launch_file, condition=None, **arguments):
    """Include `<package>/launch/<launch_file>` with the given launch arguments."""
    return IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file]),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    hardware_params = RewrittenYaml(
        source_file=LaunchConfiguration('hardware_params_file'),
        param_rewrites={'device': LaunchConfiguration('device')},
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'slam',
                default_value='true',
                choices=['true', 'false'],
                description='Run slam_toolbox (see `mode`); required by Nav2 for map->odom.',
            ),
            DeclareLaunchArgument(
                'nav',
                default_value='false',
                choices=['true', 'false'],
                description='Run the Nav2 navigation servers.',
            ),
            DeclareLaunchArgument(
                'hardware_params_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('bicycle_hardware'), 'config', 'hardware.yaml']
                ),
                description='Hardware parameters of the Arduino plugin (calibration).',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='/dev/bicycle_arduino',
                description='Serial device of the Arduino (overrides the parameter file).',
            ),
            include(
                'bicycle_description',
                'description.launch.py',
                use_sim_time='false',
                use_sim='false',
                hardware_plugin='bicycle_hardware/BicycleSystemHardware',
                hardware_params_file=hardware_params,
            ),
            include('bicycle_hardware', 'hardware.launch.py'),
            include('bicycle_control', 'control.launch.py', use_sim_time='false'),
            include(
                'bicycle_navigation',
                'slam.launch.py',
                IfCondition(LaunchConfiguration('slam')),
                use_sim_time='false',
            ),
            include(
                'bicycle_navigation',
                'navigation.launch.py',
                IfCondition(LaunchConfiguration('nav')),
                use_sim_time='false',
            ),
        ]
    )
