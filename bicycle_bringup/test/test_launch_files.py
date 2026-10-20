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

"""Smoke test of every launch file in the repository: it loads and declares its arguments."""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest

PACKAGES = (
    'bicycle_description',
    'bicycle_gazebo',
    'bicycle_control',
    'bicycle_navigation',
    'bicycle_bringup',
)

# The arguments each launch file declares itself (included files add their own).
LAUNCH_FILES = {
    ('bicycle_description', 'description.launch.py'): {
        'use_sim_time',
        'use_sim',
        'hardware_plugin',
        'controllers_file',
    },
    ('bicycle_description', 'display.launch.py'): set(),
    ('bicycle_gazebo', 'sim.launch.py'): {'world', 'headless', 'x', 'y', 'yaw'},
    ('bicycle_control', 'control.launch.py'): {'use_sim_time', 'controller_manager_timeout'},
    ('bicycle_navigation', 'slam.launch.py'): {'use_sim_time', 'mode', 'map'},
    ('bicycle_navigation', 'navigation.launch.py'): {
        'use_sim_time',
        'params_file',
        'autostart',
        'log_level',
    },
    ('bicycle_bringup', 'sim_bringup.launch.py'): {'slam', 'nav', 'rviz'},
}


def launch_file_path(package, launch_file):
    return os.path.join(get_package_share_directory(package), 'launch', launch_file)


def test_every_launch_file_is_covered():
    installed = {
        (package, os.path.basename(path))
        for package in PACKAGES
        for path in glob.glob(launch_file_path(package, '*.launch.py'))
    }
    assert installed == set(LAUNCH_FILES)


@pytest.mark.parametrize(('package', 'launch_file'), sorted(LAUNCH_FILES))
def test_launch_file_loads_and_declares_its_arguments(package, launch_file):
    source = PythonLaunchDescriptionSource(launch_file_path(package, launch_file))
    description = source.get_launch_description(LaunchContext())
    declared = {
        entity.name for entity in description.entities if isinstance(entity, DeclareLaunchArgument)
    }
    assert declared == LAUNCH_FILES[(package, launch_file)]
