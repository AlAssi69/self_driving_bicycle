// Copyright 2021 ros2_control Development Team
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

#include "include/bicycledrive_arduino/bicyclebot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bicycledrive_arduino
{
  hardware_interface::CallbackReturn BicycleDriveArduinoHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.front_wheel_name = info_.hardware_parameters["front_wheel_name"];
    cfg_.rear_wheel_name = info_.hardware_parameters["rear_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    wheel_f_.setup(cfg_.front_wheel_name, cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.rear_wheel_name, cfg_.enc_counts_per_rev);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // BicycleBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BicycleDriveArduinoHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      // {
      //   RCLCPP_FATAL(
      //       rclcpp::get_logger("BicycleDriveArduinoHardware"),
      //       "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
      //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BicycleDriveArduinoHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BicycleDriveArduinoHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BicycleDriveArduinoHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> BicycleDriveArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_f_.name, hardware_interface::HW_IF_POSITION, &wheel_f_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> BicycleDriveArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_f_.name, hardware_interface::HW_IF_POSITION, &wheel_f_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn BicycleDriveArduinoHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Configuring ...please wait...");

    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BicycleDriveArduinoHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BicycleDriveArduinoHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BicycleDriveArduinoHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BicycleDriveArduinoHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Reading...!");

    comms_.read_encoder_values(wheel_r_.enc, wheel_f_.enc);

    double delta_seconds = period.seconds();

    double pos_prev = wheel_f_.pos;
    wheel_f_.pos = wheel_f_.calc_enc_angle();
    wheel_f_.vel = (wheel_f_.pos - pos_prev) / delta_seconds;

    pos_prev = wheel_r_.pos;
    wheel_r_.pos = wheel_r_.calc_enc_angle();

    wheel_r_.pos = wheel_r_.enc * 0.2;

    wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BicycleDriveArduinoHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"), "Writing...!");

    // RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"),
    //             "Wheel F cmd is '%f'.", wheel_f_.cmd);
    // RCLCPP_INFO(rclcpp::get_logger("BicycleDriveArduinoHardware"),
    //             "Wheel R cmd is '%f'.", wheel_r_.cmd);

    int motor_f_counts_per_loop = 0;
    int motor_r_counts_per_loop = 0;

    motor_r_counts_per_loop = wheel_r_.cmd * 2; // 60;

    comms_.set_motor_values(motor_f_counts_per_loop, motor_r_counts_per_loop);

    return hardware_interface::return_type::OK;
  }

} // namespace bicycledrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    bicycledrive_arduino::BicycleDriveArduinoHardware, hardware_interface::SystemInterface)
