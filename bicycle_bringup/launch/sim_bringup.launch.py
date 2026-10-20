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
Bring up the simulated bicycle: Gazebo, description, controllers and optionally SLAM, Nav2, RViz.

This file only composes the component launch files. Their own arguments (world, headless,
x, y, yaw, mode, map, ...) are forwarded automatically when given on the command line;
`ros2 launch bicycle_bringup sim_bringup.launch.py --show-args` lists all of them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None, **arguments):
    """Include `<package>/launch/<launch_file>` with the given launch arguments."""
    return IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file]),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('bicycle_control'), 'config', 'bicycle_controller.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare('bicycle_bringup'),
            'rviz',
            IfElseSubstitution(nav, 'navigation.rviz', 'slam.rviz'),
        ]
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
                'rviz', default_value='true', choices=['true', 'false'], description='Start RViz.'
            ),
            include(
                'bicycle_description',
                'description.launch.py',
                use_sim_time='true',
                use_sim='true',
                controllers_file=controllers_file,
            ),
            include('bicycle_gazebo', 'sim.launch.py'),
            include('bicycle_control', 'control.launch.py', use_sim_time='true'),
            include(
                'bicycle_navigation', 'slam.launch.py', IfCondition(slam), use_sim_time='true'
            ),
            include(
                'bicycle_navigation', 'navigation.launch.py', IfCondition(nav), use_sim_time='true'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ]
    )
