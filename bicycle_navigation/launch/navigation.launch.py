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
Run the Nav2 servers used by the bicycle and their lifecycle manager.

Localization is not included: slam_toolbox (slam.launch.py) provides /map and map->odom.
Only the servers the bicycle's behavior trees need are started, so the bringup does not
depend on upstream additions such as the docking or route servers.

Velocity pipeline (all geometry_msgs/TwistStamped):
  controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
  -> collision_monitor -> cmd_vel -> twist_mux (bicycle_control)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

# (package, executable/node name, remappings)
NAV2_SERVERS = (
    ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_collision_monitor', 'collision_monitor', []),
    ('nav2_bt_navigator', 'bt_navigator', []),
    ('nav2_waypoint_follower', 'waypoint_follower', []),
)


def generate_launch_description():
    # allow_substs resolves $(find-pkg-share ...) used for the behavior tree paths.
    params = ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)
    log_level = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    servers = [
        Node(
            package=package,
            executable=name,
            name=name,
            output='screen',
            parameters=[params],
            arguments=log_level,
            remappings=remappings,
        )
        for package, name, remappings in NAV2_SERVERS
    ]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=log_level,
        parameters=[{
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [name for _, name, _ in NAV2_SERVERS],
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation (/clock) time source.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('bicycle_navigation'), 'config', 'nav2.yaml']),
            description='Nav2 parameter file.'),
        DeclareLaunchArgument(
            'autostart', default_value='true', choices=['true', 'false'],
            description='Configure and activate the servers automatically.'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='Log level of the Nav2 servers.'),
        GroupAction([
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
            *servers,
            lifecycle_manager,
        ]),
    ])
