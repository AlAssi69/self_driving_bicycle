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

#ifndef BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__ARDUINO_IO_HPP_
#define BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__ARDUINO_IO_HPP_

#include <stdint.h>

#include "drive_motor.hpp"
#include "firmware_core.hpp"
#include "steering_stepper.hpp"

// FirmwareIo backed by the real motor, stepper, encoder and serial port.
class ArduinoIo : public bicycle_firmware::FirmwareIo
{
public:
  ArduinoIo(DriveMotor & drive, SteeringStepper & steering);

  uint32_t now_ms() override;
  void set_drive_pwm(int32_t pwm) override;
  void set_steering_target(int32_t steps) override;
  void set_steering_motion(uint32_t max_speed, uint32_t accel) override;
  int32_t steering_position() override;
  void zero_steering() override;
  bool steering_enabled() override;
  bicycle_firmware::EncoderReading encoder() override;
  void write_line(const char * line) override;

private:
  DriveMotor & drive_;
  SteeringStepper & steering_;
};

#endif  // BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__ARDUINO_IO_HPP_
