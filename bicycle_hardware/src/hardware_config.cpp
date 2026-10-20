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

#include "bicycle_hardware/hardware_config.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <set>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "bicycle_uno/protocol.hpp"

namespace bicycle_hardware
{
namespace
{

using Params = std::unordered_map<std::string, std::string>;

// Reads hardware parameters, keeping the first error.
class ParamReader
{
public:
  ParamReader(const Params & params, std::string & error)
  : params_(params), error_(error) {}

  std::string text(const std::string & key)
  {
    const auto it = params_.find(key);
    if (it == params_.end() || it->second.empty()) {
      fail("missing hardware parameter '" + key + "'");
      return {};
    }
    return it->second;
  }

  // A number in [min, max] (min excluded when min_exclusive).
  double number(const std::string & key, double min, double max, bool min_exclusive = false)
  {
    const std::string value = text(key);
    if (value.empty()) {
      return 0.0;
    }
    double parsed = 0.0;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    const bool below = min_exclusive ? parsed <= min : parsed < min;
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
      !std::isfinite(parsed) || below || parsed > max)
    {
      fail(
        "hardware parameter '" + key + "' = '" + value + "' must be a number in " +
        (min_exclusive ? "(" : "[") + std::to_string(min) + ", " + std::to_string(max) + "]");
    }
    return parsed;
  }

  bool ok() const {return error_.empty();}

  void fail(const std::string & message)
  {
    if (error_.empty()) {
      error_ = message;
    }
  }

private:
  const Params & params_;
  std::string & error_;
};

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const auto & interface) {return interface.name == name;});
}

std::optional<JointNames> find_joints(
  const hardware_interface::HardwareInfo & info, ParamReader & reader)
{
  std::vector<std::string> steering;
  std::vector<std::string> traction;
  std::vector<std::string> passive;
  for (const auto & joint : info.joints) {
    if (!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      reader.fail("joint '" + joint.name + "' needs position and velocity state interfaces");
      return std::nullopt;
    }
    const auto & commands = joint.command_interfaces;
    const bool position = commands.size() == 1 &&
      commands[0].name == hardware_interface::HW_IF_POSITION;
    const bool velocity = commands.size() == 1 &&
      commands[0].name == hardware_interface::HW_IF_VELOCITY;
    if (commands.empty()) {
      passive.push_back(joint.name);
    } else if (position) {
      steering.push_back(joint.name);
    } else if (velocity) {
      traction.push_back(joint.name);
    } else {
      reader.fail(
        "joint '" + joint.name + "' must have one position or one velocity command "
        "interface, or none");
      return std::nullopt;
    }
  }
  if (steering.size() != 1 || traction.size() != 1 || passive.size() != 1) {
    reader.fail(
      "expected exactly one steering (position command), one traction (velocity command) and "
      "one passive (state only) joint");
    return std::nullopt;
  }
  return JointNames{steering[0], traction[0], passive[0]};
}

}  // namespace

std::optional<HardwareConfig> parse_hardware_config(
  const hardware_interface::HardwareInfo & info, std::string & error)
{
  error.clear();
  ParamReader reader(info.hardware_parameters, error);
  HardwareConfig config{};

  const auto joints = find_joints(info, reader);
  if (!joints) {
    return std::nullopt;
  }
  config.joints = *joints;

  const auto limits = info.limits.find(config.joints.steering);
  if (limits == info.limits.end() || !limits->second.has_position_limits ||
    !limits->second.has_velocity_limits)
  {
    reader.fail(
      "joint '" + config.joints.steering + "' needs URDF position and velocity limits");
    return std::nullopt;
  }
  config.steering_max_angle =
    std::min(std::abs(limits->second.min_position), std::abs(limits->second.max_position));
  config.steering_max_velocity = limits->second.max_velocity;

  config.device = reader.text("device");
  const auto baud_rate = reader.number("baud_rate", 9600, 230400);
  static const std::set<double> kBaudRates{9600, 19200, 38400, 57600, 115200, 230400};
  if (reader.ok() && kBaudRates.count(baud_rate) == 0) {
    reader.fail("hardware parameter 'baud_rate' must be a standard rate up to 230400");
  }
  config.baud_rate = static_cast<uint32_t>(baud_rate);
  config.read_timeout = std::chrono::milliseconds(
    std::lround(reader.number("read_timeout_ms", 1, 1000)));
  config.startup_timeout = std::chrono::milliseconds(
    std::lround(reader.number("startup_timeout_ms", 100, 30000)));
  config.max_consecutive_errors =
    static_cast<int>(reader.number("max_consecutive_errors", 1, 1000));

  config.encoder_counts_per_rev = reader.number("encoder_counts_per_rev", 0, 100000, true);
  config.encoder_timeout_s = reader.number("encoder_timeout_s", 0, 60, true);

  config.steering_steps_per_rad = steps_per_radian(
    reader.number("steering_steps_per_rev", 0, 100000, true),
    reader.number("steering_microsteps", 1, 256),
    reader.number("steering_gear_ratio", 0, 1000, true));
  config.steering_max_acceleration =
    reader.number("steering_max_acceleration", 0, 1000, true);

  config.drive.feedforward = reader.number("drive_feedforward", 0, 1000);
  config.drive.deadband = reader.number("drive_deadband", 0, bicycle_protocol::kMaxPwm);
  config.drive.max_pwm = reader.number("drive_max_pwm", 0, bicycle_protocol::kMaxPwm, true);
  config.drive.kp = reader.number("drive_kp", 0, 1000);
  config.drive.ki = reader.number("drive_ki", 0, 1000);
  config.drive.integral_limit = reader.number("drive_integral_limit", 0, bicycle_protocol::kMaxPwm);

  if (!reader.ok()) {
    return std::nullopt;
  }
  return config;
}

}  // namespace bicycle_hardware
