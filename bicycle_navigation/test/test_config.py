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

"""Checks of the SLAM and Nav2 configuration against Jazzy's rules and the bicycle's limits."""

import glob
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

SHARE = get_package_share_directory('bicycle_navigation')

with open(os.path.join(SHARE, 'config', 'nav2.yaml')) as f:
    NAV2 = yaml.safe_load(f)
with open(os.path.join(SHARE, 'config', 'slam.yaml')) as f:
    SLAM = yaml.safe_load(f)['slam_toolbox']['ros__parameters']

# Nav2 nodes that publish or subscribe to velocity commands. In Jazzy each one must be
# told to use TwistStamped; a single mismatch silently breaks the command chain.
VELOCITY_NODES = ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor')
BEHAVIOR_TREES = sorted(glob.glob(os.path.join(SHARE, 'behavior_trees', '*.xml')))


def walk(node, path=()):
    """Yield (path, key, value) for every mapping entry of a YAML tree."""
    if isinstance(node, dict):
        for key, value in node.items():
            yield path, key, value
            yield from walk(value, path + (key,))
    elif isinstance(node, list):
        for item in node:
            yield from walk(item, path)


def params(node_name):
    return NAV2[node_name]['ros__parameters']


def test_use_sim_time_is_left_to_the_launch_file():
    for config in (NAV2, SLAM):
        keys = [key for _, key, _ in walk(config)]
        assert 'use_sim_time' not in keys


def test_plugin_names_use_jazzy_separator():
    plugins = [value for _, key, value in walk(NAV2) if key == 'plugin']
    assert plugins
    assert all('::' in p and '/' not in p for p in plugins), plugins


def test_default_bt_plugins_are_not_listed():
    assert 'plugin_lib_names' not in params('bt_navigator')


@pytest.mark.parametrize('node', VELOCITY_NODES)
def test_stamped_velocity_commands(node):
    assert params(node)['enable_stamped_cmd_vel'] is True


def test_no_rotation_in_place():
    follow_path = params('controller_server')['FollowPath']
    assert follow_path['use_rotate_to_heading'] is False
    assert follow_path['allow_reversing'] is False
    assert params('behavior_server')['behavior_plugins'] == ['wait']
    smoother = params('velocity_smoother')
    assert smoother['min_velocity'][0] == 0.0, 'forward only'


@pytest.mark.parametrize('tree', BEHAVIOR_TREES, ids=os.path.basename)
def test_behavior_trees_only_use_available_behaviors(tree):
    tags = {element.tag for element in ET.parse(tree).iter()}
    assert not tags & {'Spin', 'BackUp', 'DriveOnHeading', 'AssistedTeleop'}


def test_bt_navigator_uses_the_bicycle_trees():
    bt = params('bt_navigator')
    prefix = '$(find-pkg-share bicycle_navigation)/behavior_trees/'
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        assert bt[key].startswith(prefix)
        assert os.path.isfile(os.path.join(SHARE, 'behavior_trees', bt[key].removeprefix(prefix)))


def test_velocity_chain_topics():
    monitor = params('collision_monitor')
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'


def test_slam_starts_a_new_map_unless_given_one():
    assert SLAM['map_file_name'] == ''
    assert 'mode' not in SLAM, 'ignored by slam_toolbox; the launch file picks the node'
    assert 'map_start_at_dock' not in SLAM


def test_no_absolute_paths():
    for config in (NAV2, SLAM):
        for _, key, value in walk(config):
            assert not (isinstance(value, str) and value.startswith('/home/')), key
