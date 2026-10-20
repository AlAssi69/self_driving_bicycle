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

#ifndef BICYCLE_HARDWARE__BICYCLE_SYSTEM_HPP_
#define BICYCLE_HARDWARE__BICYCLE_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "bicycle_hardware/drive_model.hpp"
#include "bicycle_hardware/hardware_config.hpp"
#include "bicycle_hardware/protocol.hpp"
#include "bicycle_hardware/serial_port.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace bicycle_hardware
{

// ros2_control system of the real bicycle: the Arduino Uno running firmware/bicycle_uno,
// reached over USB serial. Steering joint: position command (stepper). Traction joint:
// velocity command (DC motor, IR encoder). Front wheel: state only, estimated from the rear
// wheel and the steering angle.
//
// Lifecycle: configure opens the port and checks the firmware version and limits; activate
// starts from the current state; deactivate, cleanup, shutdown and error stop the drive.
// read() and write() return ERROR after max_consecutive_errors failed exchanges, which makes
// the controller manager stop the controllers.
class BicycleSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BicycleSystemHardware)

  BicycleSystemHardware();
  // For tests: inject the serial link.
  explicit BicycleSystemHardware(std::unique_ptr<SerialPort> serial);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct InterfaceNames
  {
    std::string steering_position;
    std::string steering_velocity;
    std::string steering_command;
    std::string traction_position;
    std::string traction_velocity;
    std::string traction_command;
    std::string front_position;
    std::string front_velocity;
  };

  bool check_firmware(std::string & error);
  bool send_limits(std::string & error);
  std::optional<protocol::State> query();
  bool send_command(int32_t pwm, int32_t steering_steps);
  void update_state(const protocol::State & state, double dt);
  void publish_state();
  hardware_interface::return_type fail(const std::string & what);
  void stop_and_close();

  std::unique_ptr<SerialPort> serial_;
  HardwareConfig config_{};
  std::optional<WheelEncoderModel> encoder_;
  std::optional<SteeringModel> steering_;
  std::optional<DriveController> drive_;
  InterfaceNames names_;
  std::string line_;

  bool active_{false};
  uint16_t seq_{0};
  int consecutive_errors_{0};
  std::optional<int32_t> last_counts_;
  int32_t steering_steps_{0};
  double steering_angle_{0.0};
  double steering_velocity_{0.0};
  double traction_position_{0.0};
  double traction_velocity_{0.0};
  double front_position_{0.0};
};

}  // namespace bicycle_hardware

#endif  // BICYCLE_HARDWARE__BICYCLE_SYSTEM_HPP_
