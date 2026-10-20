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
Cross-package consistency checks.

bicycle_description/config/geometry.yaml is the single source of truth for the robot.
ROS parameter files cannot import it, so the values the controller, Nav2 and the real
robot's hardware and lidar configuration repeat are checked against it here, together with
the frame and topic contract between packages and the Arduino firmware's limits.
"""

import ast
import math
import os
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml


def share(package, *path):
    return os.path.join(get_package_share_directory(package), *path)


def load(package, *path):
    with open(share(package, *path)) as f:
        return yaml.safe_load(f)


GEOMETRY = load('bicycle_description', 'config', 'geometry.yaml')
CONTROLLER = load('bicycle_control', 'config', 'bicycle_controller.yaml')['bicycle_controller'][
    'ros__parameters'
]
NAV2 = load('bicycle_navigation', 'config', 'nav2.yaml')
SLAM = load('bicycle_navigation', 'config', 'slam.yaml')['slam_toolbox']['ros__parameters']
BRIDGE = load('bicycle_gazebo', 'config', 'gz_bridge.yaml')
URDF = ET.fromstring(
    xacro.process_file(
        share('bicycle_description', 'urdf', 'bicycle.urdf.xacro'),
        mappings={'use_sim': 'true', 'controllers_file': 'controllers.yaml'},
    ).toxml()
)

WHEELBASE = GEOMETRY['wheelbase']
WHEEL_RADIUS = GEOMETRY['wheel']['radius']
MAX_STEERING = GEOMETRY['steering']['max_angle']
MIN_TURNING_RADIUS = WHEELBASE / math.tan(MAX_STEERING)
MAX_SPEED = GEOMETRY['traction']['max_velocity'] * WHEEL_RADIUS
ROBOT_FRAME = 'base_footprint'


def nav2(node, *keys):
    value = NAV2[node]['ros__parameters']
    for key in keys:
        value = value[key]
    return value


def costmap(name):
    return NAV2[name][name]['ros__parameters']


def footprint(name):
    return ast.literal_eval(costmap(name)['footprint'])


def test_urdf_root_is_the_robot_frame():
    links = {link.get('name') for link in URDF.findall('link')}
    children = {j.find('child').get('link') for j in URDF.findall('joint')}
    assert links - children == {ROBOT_FRAME}


def test_controller_geometry_matches_description():
    assert CONTROLLER['wheelbase'] == pytest.approx(WHEELBASE)
    assert CONTROLLER['traction_wheel_radius'] == pytest.approx(WHEEL_RADIUS)


def test_controller_joints_exist():
    joints = {j.get('name') for j in URDF.findall('joint')}
    assert set(CONTROLLER['traction_joints_names']) <= joints
    assert set(CONTROLLER['steering_joints_names']) <= joints


def test_robot_frame_everywhere():
    assert CONTROLLER['base_frame_id'] == ROBOT_FRAME
    assert SLAM['base_frame'] == ROBOT_FRAME
    assert nav2('bt_navigator', 'robot_base_frame') == ROBOT_FRAME
    assert nav2('behavior_server', 'robot_base_frame') == ROBOT_FRAME
    assert nav2('collision_monitor', 'base_frame_id') == ROBOT_FRAME
    for name in ('local_costmap', 'global_costmap'):
        assert costmap(name)['robot_base_frame'] == ROBOT_FRAME


def test_odom_and_map_frames():
    assert CONTROLLER['odom_frame_id'] == SLAM['odom_frame'] == 'odom'
    assert SLAM['map_frame'] == 'map'
    assert costmap('local_costmap')['global_frame'] == 'odom'
    assert costmap('global_costmap')['global_frame'] == 'map'


def test_scan_topic_everywhere():
    sensor = URDF.find(".//gazebo[@reference='laser_frame']/sensor")
    assert sensor.find('topic').text == 'scan'
    assert any(b['gz_topic_name'] == b['ros_topic_name'] == '/scan' for b in BRIDGE)
    assert SLAM['scan_topic'] == '/scan'
    assert nav2('collision_monitor', 'scan', 'topic') == '/scan'
    for name in ('local_costmap', 'global_costmap'):
        assert costmap(name)['obstacle_layer']['scan']['topic'] == '/scan'


def test_slam_range_within_lidar_range():
    assert SLAM['max_laser_range'] <= GEOMETRY['lidar']['max_range']


def test_planner_respects_minimum_turning_radius():
    planner = nav2('planner_server', 'GridBased')
    assert planner['minimum_turning_radius'] >= MIN_TURNING_RADIUS
    assert planner['analytic_expansion_max_length'] >= 4 * planner['minimum_turning_radius']


def test_controller_lookahead_is_feasible():
    assert nav2('controller_server', 'FollowPath', 'min_lookahead_dist') >= MIN_TURNING_RADIUS


def test_speed_limits_within_wheel_limit():
    max_velocity = nav2('velocity_smoother', 'max_velocity')
    assert max_velocity[0] <= MAX_SPEED
    assert nav2('controller_server', 'FollowPath', 'desired_linear_vel') <= max_velocity[0]


def test_turn_rate_respects_turning_radius():
    max_vx, _, max_wz = nav2('velocity_smoother', 'max_velocity')
    turning_radius = nav2('planner_server', 'GridBased', 'minimum_turning_radius')
    assert max_wz <= max_vx / turning_radius + 1e-3


@pytest.mark.parametrize('name', ['local_costmap', 'global_costmap'])
def test_footprint_covers_the_wheels(name):
    xs = [x for x, _ in footprint(name)]
    ys = [y for _, y in footprint(name)]
    half_width = GEOMETRY['wheel']['width'] / 2
    assert min(xs) <= -WHEEL_RADIUS
    assert max(xs) >= WHEELBASE + WHEEL_RADIUS
    assert min(ys) <= -half_width and max(ys) >= half_width


def test_costmaps_share_the_footprint():
    assert footprint('local_costmap') == footprint('global_costmap')


def test_global_inflation_covers_the_footprint():
    circumscribed = max(math.hypot(x, y) for x, y in footprint('global_costmap'))
    assert costmap('global_costmap')['inflation_layer']['inflation_radius'] >= circumscribed


# Real robot: bicycle_hardware's parameters and firmware against the same sources.

HARDWARE = load('bicycle_hardware', 'config', 'hardware.yaml')
LIDAR_DRIVER = load('bicycle_hardware', 'config', 'ydlidar.yaml')['ydlidar_ros2_driver_node'][
    'ros__parameters'
]
FIRMWARE = share('bicycle_hardware', 'firmware', 'bicycle_uno')


def firmware_constant(header, pattern):
    with open(os.path.join(FIRMWARE, header)) as f:
        match = re.search(pattern, f.read(), re.DOTALL)
    assert match, f'{pattern} not found in {header}'
    return [int(value) for value in match.groups()]


def test_hardware_params_reach_the_plugin():
    real = ET.fromstring(
        xacro.process_file(
            share('bicycle_description', 'urdf', 'bicycle.urdf.xacro'),
            mappings={
                'hardware_plugin': 'bicycle_hardware/BicycleSystemHardware',
                'hardware_params_file': share('bicycle_hardware', 'config', 'hardware.yaml'),
            },
        ).toxml()
    )
    params = {p.get('name'): p.text for p in real.findall('./ros2_control/hardware/param')}
    assert params == {key: str(value) for key, value in HARDWARE.items()}


def test_serial_settings_match_the_firmware():
    (baud_rate,) = firmware_constant('protocol.hpp', r'kBaudRate = (\d+);')
    assert HARDWARE['baud_rate'] == baud_rate


def test_firmware_hard_limits_cover_the_urdf_limits():
    steps_per_rad = (
        HARDWARE['steering_steps_per_rev']
        * HARDWARE['steering_microsteps']
        * HARDWARE['steering_gear_ratio']
        / (2 * math.pi)
    )
    max_steps, max_speed, max_accel, max_pwm = firmware_constant(
        'config.hpp', r'kHardLimits = \{\s*(\d+),.*?(\d+),.*?(\d+),.*?(\d+),'
    )
    assert max_steps >= round(MAX_STEERING * steps_per_rad)
    assert max_speed >= GEOMETRY['steering']['max_velocity'] * steps_per_rad
    assert max_accel >= HARDWARE['steering_max_acceleration'] * steps_per_rad
    assert max_pwm >= HARDWARE['drive_max_pwm']


def test_drive_can_reach_the_speed_limit():
    wheel_speed = GEOMETRY['traction']['max_velocity']
    pwm = HARDWARE['drive_feedforward'] * wheel_speed + HARDWARE['drive_deadband']
    assert pwm <= HARDWARE['drive_max_pwm']


def test_lidar_driver_matches_the_simulated_lidar():
    lidar = GEOMETRY['lidar']
    assert LIDAR_DRIVER['frame_id'] == 'laser_frame'
    assert URDF.find("./link[@name='laser_frame']") is not None
    assert LIDAR_DRIVER['range_min'] == pytest.approx(lidar['min_range'])
    assert LIDAR_DRIVER['range_max'] == pytest.approx(lidar['max_range'])
    assert LIDAR_DRIVER['frequency'] == pytest.approx(lidar['update_rate'])
    assert SLAM['max_laser_range'] <= LIDAR_DRIVER['range_max']
