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

"""Static checks of the processed robot description."""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

SHARE = get_package_share_directory('bicycle_description')
XACRO_FILE = os.path.join(SHARE, 'urdf', 'bicycle.urdf.xacro')
CONTROLLERS_FILE = '/tmp/controllers.yaml'

with open(os.path.join(SHARE, 'config', 'geometry.yaml')) as f:
    GEOMETRY = yaml.safe_load(f)


def process(**mappings):
    return xacro.process_file(XACRO_FILE, mappings=mappings).toxml()


@pytest.fixture(scope='module')
def sim_urdf():
    return ET.fromstring(process(use_sim='true', controllers_file=CONTROLLERS_FILE))


@pytest.fixture(scope='module')
def mock_urdf():
    return ET.fromstring(process())


def joint(urdf, name):
    element = urdf.find(f"./joint[@name='{name}']")
    assert element is not None, f'joint {name} missing'
    return element


def origin_xyz(element):
    return [float(v) for v in element.find('origin').get('xyz').split()]


def hardware_plugin(urdf):
    return urdf.find('./ros2_control/hardware/plugin').text


def test_check_urdf_accepts_description(tmp_path):
    urdf_file = tmp_path / 'bicycle.urdf'
    urdf_file.write_text(process(use_sim='true', controllers_file=CONTROLLERS_FILE))
    result = subprocess.run(['check_urdf', str(urdf_file)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_tree_is_rooted_at_base_footprint(sim_urdf):
    links = {link.get('name') for link in sim_urdf.findall('link')}
    children = {j.find('child').get('link') for j in sim_urdf.findall('joint')}
    assert links - children == {'base_footprint'}


def test_base_link_is_the_rear_axle(sim_urdf):
    base = joint(sim_urdf, 'base_joint')
    assert base.find('parent').get('link') == 'base_footprint'
    assert origin_xyz(base) == pytest.approx([0.0, 0.0, GEOMETRY['wheel']['radius']])
    assert origin_xyz(joint(sim_urdf, 'traction_joint')) == pytest.approx([0.0, 0.0, 0.0])


def test_front_axle_is_one_wheelbase_ahead(sim_urdf):
    steering = joint(sim_urdf, 'steering_joint')
    assert origin_xyz(steering) == pytest.approx([GEOMETRY['wheelbase'], 0.0, 0.0])


def test_joint_types_and_limits(sim_urdf):
    steering = joint(sim_urdf, 'steering_joint')
    assert steering.get('type') == 'revolute'
    limit = steering.find('limit')
    max_angle = GEOMETRY['steering']['max_angle']
    assert float(limit.get('lower')) == pytest.approx(-max_angle)
    assert float(limit.get('upper')) == pytest.approx(max_angle)

    traction = joint(sim_urdf, 'traction_joint')
    assert traction.get('type') == 'continuous'
    assert float(traction.find('limit').get('velocity')) == pytest.approx(
        GEOMETRY['traction']['max_velocity']
    )

    front = joint(sim_urdf, 'front_wheel_joint')
    assert front.get('type') == 'continuous'
    assert front.find('mimic') is None, 'the front wheel is passive'

    for name in ('traction_joint', 'front_wheel_joint'):
        assert joint(sim_urdf, name).find('axis').get('xyz') == '0 1 0'


def test_lidar_scan_plane_clears_the_wheels(sim_urdf):
    scan_height = (
        origin_xyz(joint(sim_urdf, 'base_joint'))[2]
        + origin_xyz(joint(sim_urdf, 'laser_joint'))[2]
    )
    assert scan_height == pytest.approx(GEOMETRY['lidar']['scan_height'])
    assert scan_height > 2 * GEOMETRY['wheel']['radius'] + 0.1


def test_lidar_sensor(sim_urdf):
    sensor = sim_urdf.find(".//gazebo[@reference='laser_frame']/sensor")
    assert sensor.get('type') == 'gpu_lidar'
    assert sensor.find('topic').text == 'scan'
    assert sensor.find('gz_frame_id').text == 'laser_frame'
    assert float(sensor.find('lidar/range/max').text) == pytest.approx(
        GEOMETRY['lidar']['max_range']
    )


def test_ros2_control_interfaces(mock_urdf):
    def interfaces(name, kind):
        element = mock_urdf.find(f"./ros2_control/joint[@name='{name}']")
        assert element is not None, f'{name} missing from ros2_control'
        return [i.get('name') for i in element.findall(kind)]

    assert interfaces('steering_joint', 'command_interface') == ['position']
    assert interfaces('traction_joint', 'command_interface') == ['velocity']
    assert interfaces('front_wheel_joint', 'command_interface') == []
    assert interfaces('front_wheel_joint', 'state_interface') == ['position', 'velocity']


def test_hardware_plugin_selection(sim_urdf, mock_urdf):
    assert hardware_plugin(mock_urdf) == 'mock_components/GenericSystem'
    assert hardware_plugin(sim_urdf) == 'gz_ros2_control/GazeboSimSystem'
    custom = ET.fromstring(process(hardware_plugin='my_robot/MySystem'))
    assert hardware_plugin(custom) == 'my_robot/MySystem'


def test_gazebo_plugin_only_in_simulation(sim_urdf, mock_urdf):
    def gz_plugins(urdf):
        return [
            p
            for p in urdf.findall('./gazebo/plugin')
            if p.get('filename') == 'gz_ros2_control-system'
        ]

    assert gz_plugins(mock_urdf) == []
    plugins = gz_plugins(sim_urdf)
    assert len(plugins) == 1
    assert plugins[0].find('parameters').text == CONTROLLERS_FILE
