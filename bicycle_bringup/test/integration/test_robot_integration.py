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
End-to-end test of the real-robot stack against the fake Arduino (no bicycle, no GPU needed).

fake_mcu runs the firmware's own request handling on a pseudo-terminal, and robot_bringup
connects to it as to the real Arduino: teleop -> twist_mux -> bicycle_controller ->
bicycle_hardware plugin -> serial -> firmware, and encoder/stepper state back to /odom.
Run it with:
  colcon test --packages-select bicycle_bringup --ctest-args -R test_robot_integration
"""

import math
import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
import launch
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import yaml

STARTUP_TIMEOUT = 60.0  # [s]
COMMAND_PERIOD = 0.05  # [s]
DEVICE = os.path.join(tempfile.gettempdir(), f'bicycle_fake_mcu_{os.getpid()}')

with open(
    os.path.join(get_package_share_directory('bicycle_description'), 'config', 'geometry.yaml')
) as f:
    GEOMETRY = yaml.safe_load(f)
with open(
    os.path.join(get_package_share_directory('bicycle_hardware'), 'config', 'hardware.yaml')
) as f:
    HARDWARE = yaml.safe_load(f)


def expected_pwm(speed):
    """PWM the plugin's feedforward law sends for a forward speed [m/s]."""
    wheel_speed = speed / GEOMETRY['wheel']['radius']
    return round(HARDWARE['drive_feedforward'] * wheel_speed + HARDWARE['drive_deadband'])


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    fake_mcu = Node(
        package='bicycle_hardware',
        executable='fake_mcu',
        arguments=['--link', DEVICE],
        output='screen',
    )
    robot = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('bicycle_bringup'), 'launch', 'robot_bringup.launch.py']
        ),
        launch_arguments={
            'device': DEVICE,
            'lidar': 'false',
            'slam': 'false',
            'nav': 'false',
        }.items(),
    )
    # Start the robot once the fake Arduino's terminal exists, as the real one would.
    start_robot = RegisterEventHandler(
        OnProcessIO(
            target_action=fake_mcu,
            on_stdout=lambda event: [robot] if b'listening' in event.text else None,
        )
    )
    return (
        launch.LaunchDescription([fake_mcu, start_robot, launch_testing.actions.ReadyToTest()]),
        {'fake_mcu': fake_mcu},
    )


class TestRobotStack(unittest.TestCase):
    """The tests run in name order and share one robot stack."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('robot_integration_test')
        cls.odom = []
        cls.joint_states = []
        cls.node.create_subscription(Odometry, '/odom', cls.odom.append, 10)
        cls.node.create_subscription(JointState, '/joint_states', cls.joint_states.append, 10)
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

    def drive(self, seconds, linear_x, angular_z=0.0, e_stop=None):
        """Publish teleop commands (and optionally the e-stop) for `seconds`."""
        end = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic() + COMMAND_PERIOD
                command = TwistStamped()
                command.header.stamp = self.node.get_clock().now().to_msg()
                command.twist.linear.x = linear_x
                command.twist.angular.z = angular_z
                self.cmd_pub.publish(command)
                if e_stop is not None:
                    self.lock_pub.publish(Bool(data=e_stop))
            rclpy.spin_once(self.node, timeout_sec=COMMAND_PERIOD / 2)

    def joint(self, name, field):
        state = self.joint_states[-1]
        return getattr(state, field)[state.name.index(name)]

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

    def test_2_teleop_drives_the_motor(self, proc_output, fake_mcu):
        self.spin_until(lambda: self.odom, 10.0, 'no /odom')
        start_x = self.odom[-1].pose.pose.position.x
        self.drive(6.0, 0.3)
        proc_output.assertWaitFor(
            f'pwm {expected_pwm(0.3)} ', process=fake_mcu, timeout=5, stream='stdout'
        )
        self.assertGreater(self.odom[-1].pose.pose.position.x - start_x, 0.5)
        wheel_speed = 0.3 / GEOMETRY['wheel']['radius']
        self.assertAlmostEqual(self.joint('traction_joint', 'velocity'), wheel_speed, delta=0.2)
        # Every joint has a state, so robot_state_publisher has the whole TF tree.
        self.assertGreater(self.joint('front_wheel_joint', 'position'), 0.0)

    def test_3_steering_follows_the_turn_rate(self):
        speed, turn_rate = 0.3, 0.2
        self.drive(3.0, speed, angular_z=turn_rate)
        expected = math.atan(turn_rate * GEOMETRY['wheelbase'] / speed)
        self.assertAlmostEqual(self.joint('steering_joint', 'position'), expected, delta=0.02)

    def test_4_e_stop_stops_the_motor(self):
        self.drive(1.0, 0.3)
        # Long enough for the encoder speed estimate to time out after the last pulse.
        self.drive(HARDWARE['encoder_timeout_s'] + 1.5, 0.3, e_stop=True)
        self.assertEqual(self.joint('traction_joint', 'velocity'), 0.0)
        self.assertAlmostEqual(self.odom[-1].twist.twist.linear.x, 0.0, places=6)
        self.drive(3.0, 0.3, e_stop=False)
        self.assertGreater(self.joint('traction_joint', 'velocity'), 0.5)
        self.drive(1.0, 0.0)
