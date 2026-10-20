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

#ifndef BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__STEERING_STEPPER_HPP_
#define BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__STEERING_STEPPER_HPP_

#include <AccelStepper.h>
#include <stdint.h>

// Steering stepper on a STEP/DIR/ENABLE driver, moved without blocking by AccelStepper.
// The position is the step count AccelStepper has issued (open loop); zero is the
// position at power-on, so centre the handlebar before switching on.
class SteeringStepper
{
public:
  SteeringStepper(
    uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, bool enable_active_low,
    bool invert_direction);

  // Configures the pins and enables the driver (holding torque). Call from setup().
  void begin();

  void set_motion(uint32_t max_speed, uint32_t accel);
  void move_to(int32_t steps);
  // Issues at most one step; call as often as possible.
  void run();
  int32_t position();
  // Declares the current position as straight ahead.
  void zero();
  bool enabled() const;

private:
  AccelStepper stepper_;
  uint8_t enable_pin_;
  bool enable_active_low_;
  bool invert_direction_;
  bool enabled_;
};

#endif  // BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__STEERING_STEPPER_HPP_
