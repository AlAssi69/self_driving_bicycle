// Copyright 2024-2026 Ashraf AlAssi, Mhd. Osama Keakaty, Sarah AlAhmad
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The plugin loaded through pluginlib by a ResourceManager, as in the controller manager,
// talking to the fake Arduino (the real firmware logic) over a pseudo-terminal.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "bicycle_hardware/fake_mcu.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{

using lifecycle_msgs::msg::State;

constexpr char kComponent[] = "BicycleSystem";
constexpr double kStepsPerRad = 1600 / (2 * M_PI);

std::string urdf(const std::string & device, int startup_timeout_ms = 5000)
{
  return std::string(
    R"(<?xml version="1.0"?>
<robot name="bicycle">
  <link name="base_link"/>
  <link name="rear_wheel"/>
  <link name="steering_link"/>
  <link name="front_wheel"/>
  <joint name="traction_joint" type="continuous">
    <parent link="base_link"/><child link="rear_wheel"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="steering_joint" type="revolute">
    <parent link="base_link"/><child link="steering_link"/><axis xyz="0 0 1"/>
    <limit lower="-0.7854" upper="0.7854" effort="10" velocity="2.0"/>
  </joint>
  <joint name="front_wheel_joint" type="continuous">
    <parent link="steering_link"/><child link="front_wheel"/><axis xyz="0 1 0"/>
  </joint>
  <ros2_control name="BicycleSystem" type="system">
    <hardware>
      <plugin>bicycle_hardware/BicycleSystemHardware</plugin>
      <param name="device">)")
         +
         device +
         R"(</param>
      <param name="baud_rate">115200</param>
      <param name="read_timeout_ms">20</param>
      <param name="startup_timeout_ms">)"
         +
         std::to_string(startup_timeout_ms) +
         R"(</param>
      <param name="max_consecutive_errors">5</param>
      <param name="encoder_counts_per_rev">8</param>
      <param name="encoder_timeout_s">2.0</param>
      <param name="steering_steps_per_rev">200</param>
      <param name="steering_microsteps">8</param>
      <param name="steering_gear_ratio">1.0</param>
      <param name="steering_max_acceleration">4.0</param>
      <param name="drive_feedforward">25.0</param>
      <param name="drive_deadband">25.0</param>
      <param name="drive_max_pwm">150.0</param>
      <param name="drive_kp">0.0</param>
      <param name="drive_ki">0.0</param>
      <param name="drive_integral_limit">30.0</param>
    </hardware>
    <joint name="steering_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="traction_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="front_wheel_joint">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)";
}

class BicycleSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::unique_ptr<hardware_interface::ResourceManager> make_resource_manager(
    const std::string & description)
  {
    return std::make_unique<hardware_interface::ResourceManager>(
      description, clock_, rclcpp::get_logger("test_bicycle_system"));
  }

  static bool set_state(
    hardware_interface::ResourceManager & rm, uint8_t id, const std::string & label)
  {
    rclcpp_lifecycle::State state(id, label);
    return rm.set_component_state(kComponent, state) == hardware_interface::return_type::OK;
  }

  // Runs control cycles at 100 Hz; returns false at the first failed read or write.
  bool run(hardware_interface::ResourceManager & rm, std::chrono::milliseconds duration)
  {
    const auto period = rclcpp::Duration::from_seconds(0.01);
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      if (rm.read(clock_->now(), period).result != hardware_interface::return_type::OK ||
        rm.write(clock_->now(), period).result != hardware_interface::return_type::OK)
      {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  static double state(hardware_interface::ResourceManager & rm, const std::string & name)
  {
    return rm.claim_state_interface(name).get_optional().value();
  }

  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
};

TEST_F(BicycleSystemTest, DrivesAndSteersTheBicycle)
{
  bicycle_hardware::FakeMcu mcu;
  mcu.start();
  auto rm = make_resource_manager(urdf(mcu.device()));
  ASSERT_TRUE(set_state(*rm, State::PRIMARY_STATE_INACTIVE, "inactive"));
  ASSERT_TRUE(set_state(*rm, State::PRIMARY_STATE_ACTIVE, "active"));

  auto steering = rm->claim_command_interface("steering_joint/position");
  auto traction = rm->claim_command_interface("traction_joint/velocity");
  ASSERT_TRUE(steering.set_value(0.3));
  ASSERT_TRUE(traction.set_value(1.6));
  ASSERT_TRUE(run(*rm, std::chrono::milliseconds(3000)));

  const auto driving = mcu.snapshot();
  EXPECT_EQ(driving.pwm, 65);  // 25 PWM/(rad/s) * 1.6 + 25 deadband
  EXPECT_EQ(driving.steering_target, std::lround(0.3 * kStepsPerRad));
  EXPECT_EQ(driving.steering_position, driving.steering_target);
  EXPECT_GT(driving.commands, 200u);

  EXPECT_NEAR(state(*rm, "steering_joint/position"), 0.3, 1.0 / kStepsPerRad);
  EXPECT_NEAR(state(*rm, "traction_joint/velocity"), 1.6, 0.2);
  const double rear = state(*rm, "traction_joint/position");
  // 2 pi / 8 per count, within the one count that may arrive between read and snapshot.
  EXPECT_NEAR(rear, driving.encoder_counts * M_PI / 4, M_PI / 4 + 1e-9);
  EXPECT_GT(rear, 2.0);
  EXPECT_GT(state(*rm, "front_wheel_joint/position"), rear);  // rolls 1/cos(0.3) times as far

  ASSERT_TRUE(traction.set_value(0.0));
  ASSERT_TRUE(run(*rm, std::chrono::milliseconds(100)));
  EXPECT_EQ(mcu.snapshot().pwm, 0);

  ASSERT_TRUE(traction.set_value(1.6));
  ASSERT_TRUE(run(*rm, std::chrono::milliseconds(100)));
  EXPECT_EQ(mcu.snapshot().pwm, 65);
  ASSERT_TRUE(set_state(*rm, State::PRIMARY_STATE_INACTIVE, "inactive"));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(mcu.snapshot().pwm, 0);  // deactivating stops the drive
}

TEST_F(BicycleSystemTest, ReportsAnErrorWhenTheArduinoIsUnplugged)
{
  bicycle_hardware::FakeMcu mcu;
  mcu.start();
  auto rm = make_resource_manager(urdf(mcu.device()));
  ASSERT_TRUE(set_state(*rm, State::PRIMARY_STATE_INACTIVE, "inactive"));
  ASSERT_TRUE(set_state(*rm, State::PRIMARY_STATE_ACTIVE, "active"));
  ASSERT_TRUE(run(*rm, std::chrono::milliseconds(200)));

  mcu.stop();
  EXPECT_FALSE(run(*rm, std::chrono::milliseconds(1000)));
  const auto status = rm->get_components_status();
  EXPECT_EQ(status.at(kComponent).state.id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// H10: a missing device used to throw out of on_configure and kill the controller manager.
TEST_F(BicycleSystemTest, ConfigureFailsCleanlyWithoutTheDevice)
{
  auto rm = make_resource_manager(urdf("/dev/bicycle_does_not_exist"));
  EXPECT_FALSE(set_state(*rm, State::PRIMARY_STATE_INACTIVE, "inactive"));
}

TEST_F(BicycleSystemTest, ConfigureFailsWhenTheArduinoDoesNotAnswer)
{
  bicycle_hardware::FakeMcu mcu;  // not started: nothing answers on the terminal
  auto rm = make_resource_manager(urdf(mcu.device(), 300));
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(set_state(*rm, State::PRIMARY_STATE_INACTIVE, "inactive"));
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(2));
}

}  // namespace
