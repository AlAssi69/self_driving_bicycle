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
Activate the bicycle's controllers and the velocity command multiplexer.

The controller_manager itself is provided by the hardware layer (gz_ros2_control in
simulation). twist_mux is the only writer of /bicycle_controller/reference.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawner(controller, *extra_arguments):
    """Load, configure and activate `controller` in /controller_manager."""
    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            controller,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', LaunchConfiguration('controller_manager_timeout'),
            *extra_arguments,
        ],
    )


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('bicycle_control'), 'config'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation (/clock) time source.'),
        DeclareLaunchArgument(
            'controller_manager_timeout', default_value='120',
            description='Seconds to wait for the controller_manager (Gazebo can start slowly).'),
        spawner('joint_state_broadcaster'),
        # The steering controllers publish odometry under their own namespace;
        # expose it on the standard topics.
        spawner(
            'bicycle_controller',
            '--controller-ros-args', '-r /bicycle_controller/tf_odometry:=/tf',
            '--controller-ros-args', '-r /bicycle_controller/odometry:=/odom',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[
                PathJoinSubstitution([config, 'twist_mux_topics.yaml']),
                PathJoinSubstitution([config, 'twist_mux_locks.yaml']),
                {'use_sim_time': LaunchConfiguration('use_sim_time'), 'use_stamped': True},
            ],
            remappings=[('cmd_vel_out', '/bicycle_controller/reference')],
        ),
    ])
