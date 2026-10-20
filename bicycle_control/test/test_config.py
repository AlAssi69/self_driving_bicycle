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

"""Checks of the controller and command multiplexer configuration."""

import os

from ament_index_python.packages import get_package_share_directory
import yaml

CONFIG = os.path.join(get_package_share_directory('bicycle_control'), 'config')


def load(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)


def test_controller_types():
    manager = load('bicycle_controller.yaml')['controller_manager']['ros__parameters']
    assert manager['bicycle_controller']['type'] == \
        'bicycle_steering_controller/BicycleSteeringController'
    assert manager['joint_state_broadcaster']['type'] == \
        'joint_state_broadcaster/JointStateBroadcaster'


def test_no_deprecated_or_removed_controller_parameters():
    params = load('bicycle_controller.yaml')['bicycle_controller']['ros__parameters']
    removed = {'use_stamped_vel', 'in_chained_mode', 'front_wheels_names', 'rear_wheels_names',
               'front_wheel_radius', 'rear_wheel_radius', 'front_steering'}
    assert not removed & params.keys()


def test_controller_stops_without_commands():
    params = load('bicycle_controller.yaml')['bicycle_controller']['ros__parameters']
    assert 0.0 < params['reference_timeout'] <= 1.0


def test_e_stop_overrides_every_command_source():
    topics = load('twist_mux_topics.yaml')['twist_mux']['ros__parameters']['topics']
    locks = load('twist_mux_locks.yaml')['twist_mux']['ros__parameters']['locks']
    assert locks['e_stop']['priority'] > max(t['priority'] for t in topics.values())
    assert locks['e_stop']['timeout'] == 0.0, 'the e-stop must stay latched'


def test_teleop_overrides_navigation():
    topics = load('twist_mux_topics.yaml')['twist_mux']['ros__parameters']['topics']
    assert topics['teleop']['priority'] > topics['navigation']['priority']
