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

#ifndef BICYCLE_HARDWARE__HARDWARE_CONFIG_HPP_
#define BICYCLE_HARDWARE__HARDWARE_CONFIG_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "bicycle_hardware/drive_model.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace bicycle_hardware
{

// Joints of the ros2_control system, found by their interfaces.
struct JointNames
{
  std::string steering;     // position command
  std::string traction;     // velocity command
  std::string front_wheel;  // state only (passive)
};

// Everything the plugin needs, validated. Parameters come from the <hardware> params
// (bicycle_hardware/config/hardware.yaml); steering limits from the URDF joint limits.
struct HardwareConfig
{
  JointNames joints;

  std::string device;
  uint32_t baud_rate;
  std::chrono::milliseconds read_timeout;
  std::chrono::milliseconds startup_timeout;
  int max_consecutive_errors;

  double encoder_counts_per_rev;
  double encoder_timeout_s;

  double steering_steps_per_rad;
  double steering_max_angle;         // rad, URDF position limit
  double steering_max_velocity;      // rad/s, URDF velocity limit
  double steering_max_acceleration;  // rad/s^2

  DriveGains drive;
};

// Builds the configuration; on failure returns std::nullopt and the reason in `error`.
std::optional<HardwareConfig> parse_hardware_config(
  const hardware_interface::HardwareInfo & info, std::string & error);

}  // namespace bicycle_hardware

#endif  // BICYCLE_HARDWARE__HARDWARE_CONFIG_HPP_
