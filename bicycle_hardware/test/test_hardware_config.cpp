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

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "bicycle_hardware/hardware_config.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

hardware_interface::ComponentInfo joint(const std::string & name, const char * command)
{
  hardware_interface::ComponentInfo info;
  info.name = name;
  info.type = "joint";
  if (command != nullptr) {
    hardware_interface::InterfaceInfo interface;
    interface.name = command;
    info.command_interfaces.push_back(interface);
  }
  for (const char * state : {HW_IF_POSITION, HW_IF_VELOCITY}) {
    hardware_interface::InterfaceInfo interface;
    interface.name = state;
    info.state_interfaces.push_back(interface);
  }
  return info;
}

hardware_interface::HardwareInfo valid_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "BicycleSystem";
  info.joints = {
    joint("traction_joint", HW_IF_VELOCITY),
    joint("steering_joint", HW_IF_POSITION),
    joint("front_wheel_joint", nullptr),
  };
  joint_limits::JointLimits steering;
  steering.has_position_limits = true;
  steering.min_position = -0.7854;
  steering.max_position = 0.7854;
  steering.has_velocity_limits = true;
  steering.max_velocity = 2.0;
  info.limits["steering_joint"] = steering;
  info.hardware_parameters = {
    {"device", "/dev/bicycle_arduino"},
    {"baud_rate", "115200"},
    {"read_timeout_ms", "20"},
    {"startup_timeout_ms", "5000"},
    {"max_consecutive_errors", "5"},
    {"encoder_counts_per_rev", "8"},
    {"encoder_timeout_s", "2.0"},
    {"steering_steps_per_rev", "200"},
    {"steering_microsteps", "8"},
    {"steering_gear_ratio", "1.0"},
    {"steering_max_acceleration", "4.0"},
    {"drive_feedforward", "25.0"},
    {"drive_deadband", "25.0"},
    {"drive_max_pwm", "150.0"},
    {"drive_kp", "0.0"},
    {"drive_ki", "0.0"},
    {"drive_integral_limit", "30.0"},
  };
  return info;
}

}  // namespace

TEST(HardwareConfig, ParsesValidConfiguration)
{
  std::string error;
  const auto config = bicycle_hardware::parse_hardware_config(valid_info(), error);
  ASSERT_TRUE(config) << error;
  EXPECT_EQ(config->joints.steering, "steering_joint");
  EXPECT_EQ(config->joints.traction, "traction_joint");
  EXPECT_EQ(config->joints.front_wheel, "front_wheel_joint");
  EXPECT_EQ(config->device, "/dev/bicycle_arduino");
  EXPECT_EQ(config->baud_rate, 115200u);
  EXPECT_EQ(config->read_timeout.count(), 20);
  EXPECT_EQ(config->max_consecutive_errors, 5);
  EXPECT_DOUBLE_EQ(config->steering_max_angle, 0.7854);
  EXPECT_DOUBLE_EQ(config->steering_max_velocity, 2.0);
  EXPECT_NEAR(config->steering_steps_per_rad, 1600 / (2 * M_PI), 1e-9);
  EXPECT_DOUBLE_EQ(config->drive.feedforward, 25.0);
}

// H9: missing or bad values used to throw from std::stoi and kill the controller manager.
TEST(HardwareConfig, ReportsMissingParameter)
{
  auto info = valid_info();
  info.hardware_parameters.erase("encoder_counts_per_rev");
  std::string error;
  EXPECT_FALSE(bicycle_hardware::parse_hardware_config(info, error));
  EXPECT_NE(error.find("encoder_counts_per_rev"), std::string::npos) << error;
}

TEST(HardwareConfig, RejectsBadValues)
{
  const std::vector<std::pair<std::string, std::string>> bad_values = {
    {"encoder_counts_per_rev", "0"},
    {"encoder_counts_per_rev", "eight"},
    {"encoder_counts_per_rev", "8 counts"},
    {"baud_rate", "100000"},
    {"read_timeout_ms", "0"},
    {"drive_max_pwm", "300"},
    {"steering_microsteps", "nan"},
    {"device", ""},
  };
  for (const auto & [key, value] : bad_values) {
    auto info = valid_info();
    info.hardware_parameters[key] = value;
    std::string error;
    EXPECT_FALSE(bicycle_hardware::parse_hardware_config(info, error)) << key << "=" << value;
    EXPECT_NE(error.find(key), std::string::npos) << error;
  }
}

TEST(HardwareConfig, RequiresOneJointOfEachRole)
{
  auto info = valid_info();
  info.joints.pop_back();
  std::string error;
  EXPECT_FALSE(bicycle_hardware::parse_hardware_config(info, error));
  EXPECT_NE(error.find("exactly one"), std::string::npos) << error;

  info = valid_info();
  info.joints[0].state_interfaces.pop_back();
  EXPECT_FALSE(bicycle_hardware::parse_hardware_config(info, error));
  EXPECT_NE(error.find("traction_joint"), std::string::npos) << error;
}

TEST(HardwareConfig, RequiresSteeringLimits)
{
  auto info = valid_info();
  info.limits.clear();
  std::string error;
  EXPECT_FALSE(bicycle_hardware::parse_hardware_config(info, error));
  EXPECT_NE(error.find("limits"), std::string::npos) << error;
}
