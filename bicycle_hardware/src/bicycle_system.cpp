// Copyright 2021 ros2_control Development Team
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

#include "bicycle_hardware/bicycle_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bicycle_hardware
{
namespace
{

using CallbackReturn = hardware_interface::CallbackReturn;
using Clock = std::chrono::steady_clock;

// Retry period of the version request while the Uno boots (opening the port resets it).
constexpr std::chrono::milliseconds kHandshakeRetry{250};
constexpr int kThrottleMs = 2000;

std::chrono::milliseconds remaining(Clock::time_point deadline)
{
  return std::max(
    std::chrono::milliseconds(0),
    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()));
}

}  // namespace

BicycleSystemHardware::BicycleSystemHardware()
: BicycleSystemHardware(std::make_unique<PosixSerialPort>())
{
}

BicycleSystemHardware::BicycleSystemHardware(std::unique_ptr<SerialPort> serial)
: serial_(std::move(serial))
{
  line_.reserve(2 * bicycle_protocol::kMaxLineLength);
}

CallbackReturn BicycleSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  std::string error;
  const auto config = parse_hardware_config(info_, error);
  if (!config) {
    RCLCPP_FATAL(get_logger(), "Invalid configuration: %s", error.c_str());
    return CallbackReturn::ERROR;
  }
  config_ = *config;
  encoder_.emplace(config_.encoder_counts_per_rev, config_.encoder_timeout_s);
  steering_.emplace(config_.steering_steps_per_rad);
  drive_.emplace(config_.drive);

  const auto name = [](const std::string & joint, const char * interface) {
      return joint + "/" + interface;
    };
  const auto & joints = config_.joints;
  names_.steering_position = name(joints.steering, hardware_interface::HW_IF_POSITION);
  names_.steering_velocity = name(joints.steering, hardware_interface::HW_IF_VELOCITY);
  names_.steering_command = name(joints.steering, hardware_interface::HW_IF_POSITION);
  names_.traction_position = name(joints.traction, hardware_interface::HW_IF_POSITION);
  names_.traction_velocity = name(joints.traction, hardware_interface::HW_IF_VELOCITY);
  names_.traction_command = name(joints.traction, hardware_interface::HW_IF_VELOCITY);
  names_.front_position = name(joints.front_wheel, hardware_interface::HW_IF_POSITION);
  names_.front_velocity = name(joints.front_wheel, hardware_interface::HW_IF_VELOCITY);
  return CallbackReturn::SUCCESS;
}

CallbackReturn BicycleSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    serial_->open(config_.device, config_.baud_rate);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Cannot open the Arduino serial port '%s': %s", config_.device.c_str(),
      e.what());
    return CallbackReturn::ERROR;
  }

  std::string error;
  try {
    if (!check_firmware(error) || !send_limits(error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      serial_->close();
      return CallbackReturn::ERROR;
    }
    seq_ = 0;
    last_counts_.reset();
    traction_position_ = 0.0;
    front_position_ = 0.0;
    const auto state = query();
    if (!state) {
      RCLCPP_ERROR(get_logger(), "The Arduino does not answer state queries");
      serial_->close();
      return CallbackReturn::ERROR;
    }
    update_state(*state, 0.0);
    publish_state();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Serial link failed while configuring: %s", e.what());
    serial_->close();
    return CallbackReturn::ERROR;
  }
  consecutive_errors_ = 0;
  RCLCPP_INFO(
    get_logger(), "Connected to %s on %s", bicycle_protocol::kFirmwareName,
    config_.device.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn BicycleSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_and_close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn BicycleSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // Start from the current state, so the steering does not jump and the bike stands still.
  drive_->reset();
  set_command(names_.steering_command, steering_angle_);
  set_command(names_.traction_command, 0.0);
  try {
    if (!send_command(0, steering_steps_)) {
      RCLCPP_ERROR(get_logger(), "Cannot send the first command to the Arduino");
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Serial link failed while activating: %s", e.what());
    return CallbackReturn::ERROR;
  }
  consecutive_errors_ = 0;
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn BicycleSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  drive_->reset();
  try {
    send_command(0, steering_steps_);
  } catch (const std::exception & e) {
    // The firmware watchdog stops the drive anyway.
    RCLCPP_WARN(get_logger(), "Cannot send the stop command: %s", e.what());
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BicycleSystemHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_and_close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn BicycleSystemHardware::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "Stopping the bicycle after a hardware error");
  stop_and_close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BicycleSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!serial_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }
  std::optional<protocol::State> state;
  try {
    state = query();
  } catch (const std::exception & e) {
    return fail(e.what());
  }
  if (!state) {
    return fail("no state reply from the Arduino");
  }
  consecutive_errors_ = 0;
  update_state(*state, period.seconds());
  publish_state();

  if (active_ && (state->flags & bicycle_protocol::kFlagWatchdog)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs,
      "The firmware watchdog stopped the drive: commands arrive too late");
  }
  if (state->flags & bicycle_protocol::kFlagSteeringClamped) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs,
      "The firmware clamped the steering target to its hard limit (config.hpp)");
  }
  if (state->flags & bicycle_protocol::kFlagCommandError) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs, "The firmware rejected a command");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BicycleSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!serial_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }
  double steering_target = get_command(names_.steering_command);
  if (!std::isfinite(steering_target)) {
    steering_target = steering_angle_;
  }
  steering_target =
    std::clamp(steering_target, -config_.steering_max_angle, config_.steering_max_angle);
  const int32_t pwm =
    drive_->update(get_command(names_.traction_command), traction_velocity_, period.seconds());
  try {
    if (!send_command(pwm, steering_->to_steps(steering_target))) {
      return fail("command write timed out");
    }
  } catch (const std::exception & e) {
    return fail(e.what());
  }
  return hardware_interface::return_type::OK;
}

bool BicycleSystemHardware::check_firmware(std::string & error)
{
  const auto deadline = Clock::now() + config_.startup_timeout;
  std::optional<protocol::FirmwareVersion> version;
  while (!version && Clock::now() < deadline) {
    if (!serial_->write(protocol::version_request().view(), config_.read_timeout)) {
      continue;
    }
    while (!version && serial_->read_line(line_, std::min(kHandshakeRetry, remaining(deadline)))) {
      version = protocol::parse_version(line_);  // anything else is boot noise
    }
  }
  if (!version) {
    error = "No answer from the Arduino on " + config_.device + " within " +
      std::to_string(config_.startup_timeout.count()) + " ms: is firmware/bicycle_uno flashed?";
    return false;
  }
  if (version->name != bicycle_protocol::kFirmwareName ||
    version->version != bicycle_protocol::kVersion)
  {
    error = "Firmware '" + version->name + "' version " + std::to_string(version->version) +
      " found, '" + bicycle_protocol::kFirmwareName + "' version " +
      std::to_string(bicycle_protocol::kVersion) + " expected: reflash firmware/bicycle_uno";
    return false;
  }
  return true;
}

bool BicycleSystemHardware::send_limits(std::string & error)
{
  const double steps_per_rad = steering_->steps_per_rad();
  const int32_t max_steps = steering_->to_steps(config_.steering_max_angle);
  const auto speed = static_cast<uint32_t>(
    std::lround(config_.steering_max_velocity * steps_per_rad));
  const auto accel = static_cast<uint32_t>(
    std::lround(config_.steering_max_acceleration * steps_per_rad));

  serial_->discard_input();
  if (!serial_->write(
      protocol::limits_request(max_steps, speed, accel).view(), config_.read_timeout))
  {
    error = "Cannot send the steering limits to the Arduino";
    return false;
  }
  const auto deadline = Clock::now() + config_.read_timeout * 10;
  while (serial_->read_line(line_, remaining(deadline))) {
    if (const auto code = protocol::parse_error(line_)) {
      error = "The Arduino rejected the steering limits (error " + std::to_string(*code) + ")";
      return false;
    }
    const auto limits = protocol::parse_limits(line_);
    if (!limits) {
      continue;
    }
    if (limits->max_steps < max_steps || limits->max_speed < speed || limits->accel < accel) {
      RCLCPP_WARN(
        get_logger(),
        "The firmware hard limits (config.hpp) are tighter than the URDF: steering %.3f rad at "
        "%.2f rad/s instead of %.3f rad at %.2f rad/s",
        steering_->to_radians(limits->max_steps), limits->max_speed / steps_per_rad,
        config_.steering_max_angle, config_.steering_max_velocity);
    }
    return true;
  }
  error = "The Arduino did not acknowledge the steering limits";
  return false;
}

std::optional<protocol::State> BicycleSystemHardware::query()
{
  const uint16_t seq = ++seq_;
  if (!serial_->write(protocol::query_request(seq).view(), config_.read_timeout)) {
    return std::nullopt;
  }
  const auto deadline = Clock::now() + config_.read_timeout;
  while (serial_->read_line(line_, remaining(deadline))) {
    const auto state = protocol::parse_state(line_);
    if (state && state->seq == seq) {
      return state;
    }
    // A late reply to an earlier query, or noise: skip it.
  }
  return std::nullopt;
}

bool BicycleSystemHardware::send_command(int32_t pwm, int32_t steering_steps)
{
  return serial_->write(
    protocol::command_request(pwm, steering_steps).view(), config_.read_timeout);
}

void BicycleSystemHardware::update_state(const protocol::State & state, double dt)
{
  // Integrate count deltas, so positions stay continuous when the Arduino restarts.
  if (last_counts_) {
    const double delta = encoder_->to_radians(
      static_cast<int64_t>(state.encoder_counts) - *last_counts_);
    traction_position_ += delta;
    front_position_ += delta * front_wheel_ratio(steering_angle_);
  }
  last_counts_ = state.encoder_counts;
  traction_velocity_ = encoder_->velocity(state.pulse_period_us);

  const double angle = steering_->to_radians(state.steering_steps);
  steering_velocity_ = dt > 0.0 ? (angle - steering_angle_) / dt : 0.0;
  steering_angle_ = angle;
  steering_steps_ = state.steering_steps;
}

void BicycleSystemHardware::publish_state()
{
  set_state(names_.steering_position, steering_angle_);
  set_state(names_.steering_velocity, steering_velocity_);
  set_state(names_.traction_position, traction_position_);
  set_state(names_.traction_velocity, traction_velocity_);
  set_state(names_.front_position, front_position_);
  set_state(names_.front_velocity, traction_velocity_ * front_wheel_ratio(steering_angle_));
}

hardware_interface::return_type BicycleSystemHardware::fail(const std::string & what)
{
  ++consecutive_errors_;
  if (consecutive_errors_ >= config_.max_consecutive_errors) {
    RCLCPP_ERROR(
      get_logger(), "Serial link to the Arduino lost after %d failed exchanges: %s",
      consecutive_errors_, what.c_str());
    return hardware_interface::return_type::ERROR;
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs, "Serial exchange failed (%d/%d): %s",
    consecutive_errors_, config_.max_consecutive_errors, what.c_str());
  return hardware_interface::return_type::OK;
}

void BicycleSystemHardware::stop_and_close()
{
  active_ = false;
  if (!serial_->is_open()) {
    return;
  }
  try {
    send_command(0, steering_steps_);
  } catch (const std::exception &) {
    // Best effort: the firmware watchdog stops the drive anyway.
  }
  serial_->close();
}

}  // namespace bicycle_hardware

PLUGINLIB_EXPORT_CLASS(
  bicycle_hardware::BicycleSystemHardware, hardware_interface::SystemInterface)
