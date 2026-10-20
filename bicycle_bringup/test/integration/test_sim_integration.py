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
Headless end-to-end test: Gazebo, ros2_control, twist_mux and the lidar bridge.

Run it with:
  colcon test --packages-select bicycle_bringup --ctest-args -L sim_integration
"""

import math
import re
import subprocess
import time
import unittest

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

STARTUP_TIMEOUT = 180.0  # [s] Gazebo with software rendering can be slow to start
COMMAND_PERIOD = 0.05  # [s]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('bicycle_bringup'), 'launch', 'sim_bringup.launch.py']
                ),
                launch_arguments={
                    'headless': 'true',
                    'slam': 'false',
                    'nav': 'false',
                    'rviz': 'false',
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def gazebo_roll_pitch(model='bicycle', world='obstacles_course'):
    """Return the model's roll and pitch [rad] from Gazebo's ground-truth pose topic."""
    text = subprocess.run(
        ['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{world}/pose/info'],
        capture_output=True,
        text=True,
        timeout=20,
    ).stdout
    match = re.search(
        r'name: "' + model + r'"\s*id: \d+\s*position \{[^}]*\}\s*orientation \{([^}]*)\}', text
    )
    assert match, f'{model} not found in /world/{world}/pose/info'
    q = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    q.update({k: float(v) for k, v in re.findall(r'(\w): ([-\d.e]+)', match.group(1))})
    roll = math.atan2(2 * (q['w'] * q['x'] + q['y'] * q['z']), 1 - 2 * (q['x'] ** 2 + q['y'] ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q['w'] * q['y'] - q['z'] * q['x']))))
    return roll, pitch


class TestSimulatedBicycle(unittest.TestCase):
    """The tests run in name order and share one simulation."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'sim_integration_test',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        cls.scans = []
        cls.odom = []
        cls.node.create_subscription(LaserScan, '/scan', cls.scans.append, 10)
        cls.node.create_subscription(Odometry, '/odom', cls.odom.append, 10)
        cls.cmd_pub = cls.node.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        cls.lock_pub = cls.node.create_publisher(Bool, '/e_stop', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout, message):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(message)

    def drive(self, seconds, linear_x, e_stop=None):
        """Publish teleop commands (and optionally the e-stop) for `seconds` of wall time."""
        end = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic() + COMMAND_PERIOD
                command = TwistStamped()
                command.header.stamp = self.node.get_clock().now().to_msg()
                command.twist.linear.x = linear_x
                self.cmd_pub.publish(command)
                if e_stop is not None:
                    self.lock_pub.publish(Bool(data=e_stop))
            rclpy.spin_once(self.node, timeout_sec=COMMAND_PERIOD / 2)

    def recent_speed(self, samples=10):
        speeds = [m.twist.twist.linear.x for m in self.odom[-samples:]]
        return sum(speeds) / len(speeds)

    def test_1_controllers_become_active(self):
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(
            client.wait_for_service(timeout_sec=STARTUP_TIMEOUT),
            'controller_manager did not start',
        )

        def all_active():
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            states = (
                {c.name: c.state for c in future.result().controller} if future.result() else {}
            )
            return (
                states.get('joint_state_broadcaster') == 'active'
                and states.get('bicycle_controller') == 'active'
            )

        self.spin_until(all_active, STARTUP_TIMEOUT, 'controllers did not become active')

    def test_2_lidar_publishes_scans(self):
        self.spin_until(lambda: len(self.scans) >= 20, 30.0, 'too few /scan messages')
        scan = self.scans[-1]
        self.assertEqual(scan.header.frame_id, 'laser_frame')
        self.assertEqual(len(scan.ranges), 360)
        hits = [r for r in scan.ranges if math.isfinite(r)]
        self.assertGreater(len(hits), 180, 'the arena walls should be visible')

    def test_3_teleop_drives_forward(self):
        self.spin_until(lambda: self.odom, 30.0, 'no /odom')
        start_x = self.odom[-1].pose.pose.position.x
        self.drive(4.0, 0.3)
        travelled = self.odom[-1].pose.pose.position.x - start_x
        self.assertGreater(travelled, 0.5)
        self.assertAlmostEqual(self.recent_speed(), 0.3, delta=0.05)

    def test_4_e_stop_overrides_teleop(self):
        self.drive(2.0, 0.3)
        self.drive(3.0, 0.3, e_stop=True)
        self.assertLess(abs(self.recent_speed()), 0.02, 'e_stop did not stop the bicycle')
        self.drive(3.0, 0.3, e_stop=False)
        self.assertAlmostEqual(self.recent_speed(), 0.3, delta=0.05)
        self.drive(1.0, 0.0)

    def test_5_bicycle_stays_upright(self):
        roll, pitch = gazebo_roll_pitch()
        self.assertLess(abs(math.degrees(roll)), 5.0)
        self.assertLess(abs(math.degrees(pitch)), 5.0)
