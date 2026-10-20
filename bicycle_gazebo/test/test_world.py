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

"""Checks of the Gazebo worlds, models and bridge configuration."""

import glob
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

SHARE = get_package_share_directory('bicycle_gazebo')
WORLDS = sorted(glob.glob(os.path.join(SHARE, 'worlds', '*.sdf')))
MODELS_DIR = os.path.join(SHARE, 'models')
REQUIRED_SYSTEMS = {
    'gz::sim::systems::Physics',
    'gz::sim::systems::UserCommands',
    'gz::sim::systems::SceneBroadcaster',
    'gz::sim::systems::Sensors',  # the lidar is a rendering sensor
}


def test_worlds_exist():
    assert WORLDS


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_world_declares_required_systems(world):
    plugins = {p.get('name') for p in ET.parse(world).getroot().findall('./world/plugin')}
    assert REQUIRED_SYSTEMS <= plugins


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_world_models_are_vendored(world):
    uris = [
        (u.text or '').strip() for u in ET.parse(world).getroot().findall('./world/include/uri')
    ]
    for uri in uris:
        assert uri.startswith('model://'), f'{uri}: only vendored models (offline, CI)'
        assert os.path.isfile(
            os.path.join(MODELS_DIR, uri.removeprefix('model://'), 'model.config')
        )


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_world_has_no_gazebo_classic_leftovers(world):
    text = open(world).read()
    assert 'gazebo.material' not in text
    assert '<state' not in text


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz CLI not available')
@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_world_is_valid_sdf(world):
    env = dict(os.environ, SDF_PATH=MODELS_DIR)  # how standalone sdformat resolves model://
    result = subprocess.run(['gz', 'sdf', '-k', world], capture_output=True, text=True, env=env)
    assert result.returncode == 0 and 'Valid' in result.stdout, result.stdout + result.stderr


def test_bridge_only_forwards_sensor_data_to_ros():
    with open(os.path.join(SHARE, 'config', 'gz_bridge.yaml')) as f:
        bridges = yaml.safe_load(f)
    assert {b['ros_topic_name'] for b in bridges} == {'/clock', '/scan'}
    assert all(b['direction'] == 'GZ_TO_ROS' for b in bridges)
