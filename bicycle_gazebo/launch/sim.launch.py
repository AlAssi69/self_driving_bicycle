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
Start Gazebo Harmonic, spawn the bicycle from /robot_description and bridge /clock and /scan.

robot_state_publisher (bicycle_description/description.launch.py with use_sim:=true) must
publish /robot_description: it is what gets spawned and what gz_ros2_control reads.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_args = [
        LaunchConfiguration('world'),
        ' -r -v 3',
        IfElseSubstitution(LaunchConfiguration('headless'), ' -s --headless-rendering', ''),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='obstacles_course.sdf',
                description='World file name (resolved via GZ_SIM_RESOURCE_PATH) or full path.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                choices=['true', 'false'],
                description='Run only the Gazebo server, without the GUI.',
            ),
            DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m].'),
            DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m].'),
            DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad].'),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                ),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-topic',
                    'robot_description',
                    '-name',
                    'bicycle',
                    '-x',
                    LaunchConfiguration('x'),
                    '-y',
                    LaunchConfiguration('y'),
                    '-z',
                    '0.0',
                    '-Y',
                    LaunchConfiguration('yaw'),
                ],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                output='screen',
                parameters=[
                    {
                        'config_file': PathJoinSubstitution(
                            [FindPackageShare('bicycle_gazebo'), 'config', 'gz_bridge.yaml']
                        ),
                    }
                ],
            ),
        ]
    )
