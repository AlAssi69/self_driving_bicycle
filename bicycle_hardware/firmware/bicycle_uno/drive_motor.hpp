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

#ifndef BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__DRIVE_MOTOR_HPP_
#define BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__DRIVE_MOTOR_HPP_

#include <stdint.h>

// DC drive motor on a BTS7960 (IBT-2) H-bridge: one PWM input per direction.
class DriveMotor
{
public:
  DriveMotor(
    uint8_t forward_pwm_pin, uint8_t reverse_pwm_pin, uint8_t forward_enable_pin,
    uint8_t reverse_enable_pin);

  // Configures the pins and stops the motor. Call from setup().
  void begin();

  // Signed duty cycle in [-255, 255]; the sign selects the direction.
  void set_pwm(int32_t pwm);

private:
  uint8_t forward_pwm_pin_;
  uint8_t reverse_pwm_pin_;
  uint8_t forward_enable_pin_;
  uint8_t reverse_enable_pin_;
};

#endif  // BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__DRIVE_MOTOR_HPP_
