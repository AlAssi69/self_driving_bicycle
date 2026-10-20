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
Run the real bicycle's hardware layer: the controller manager and the lidar driver.

The robot's counterpart of bicycle_gazebo/sim.launch.py: it provides the controller manager
(hosting the bicycle_hardware plugin) and /scan; the controllers, SLAM and Nav2 on top are the
same as in simulation. The controller manager reads the URDF from /robot_description, published
by bicycle_description/description.launch.py with
hardware_plugin:=bicycle_hardware/BicycleSystemHardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('bicycle_control'), 'config', 'bicycle_controller.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'lidar',
                default_value='true',
                choices=['true', 'false'],
                description='Start the YDLIDAR X4 Pro driver (ydlidar_ros2_driver, robot.repos).',
            ),
            DeclareLaunchArgument(
                'lidar_params_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('bicycle_hardware'), 'config', 'ydlidar.yaml']
                ),
                description='ydlidar_ros2_driver parameter file.',
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[controllers_file],
            ),
            Node(
                package='ydlidar_ros2_driver',
                executable='ydlidar_ros2_driver_node',
                name='ydlidar_ros2_driver_node',
                output='screen',
                parameters=[LaunchConfiguration('lidar_params_file')],
                condition=IfCondition(LaunchConfiguration('lidar')),
            ),
        ]
    )
