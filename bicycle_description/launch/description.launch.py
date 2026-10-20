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

"""Publish /robot_description and the robot's TF tree with robot_state_publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('bicycle_description'), 'urdf', 'bicycle.urdf.xacro']
    )
    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                xacro_file,
                ' use_sim:=',
                LaunchConfiguration('use_sim'),
                ' hardware_plugin:=',
                LaunchConfiguration('hardware_plugin'),
                ' controllers_file:=',
                LaunchConfiguration('controllers_file'),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the simulation (/clock) time source.',
            ),
            DeclareLaunchArgument(
                'use_sim',
                default_value='false',
                description='Add the Gazebo ros2_control plugin to the description.',
            ),
            DeclareLaunchArgument(
                'hardware_plugin',
                default_value='',
                description='ros2_control system plugin; empty selects it from use_sim.',
            ),
            DeclareLaunchArgument(
                'controllers_file',
                default_value='',
                description='Controller parameter file loaded by gz_ros2_control (use_sim only).',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'use_sim_time': use_sim_time,
                    }
                ],
            ),
        ]
    )
