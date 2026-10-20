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

#include "steering_stepper.hpp"

SteeringStepper::SteeringStepper(
  uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, bool enable_active_low,
  bool invert_direction)
// enable = false: AccelStepper must not touch the pins before setup().
: stepper_(AccelStepper::DRIVER, step_pin, dir_pin, 0xff, 0xff, false), enable_pin_(enable_pin),
  enable_active_low_(enable_active_low), invert_direction_(invert_direction), enabled_(false)
{
}

void SteeringStepper::begin()
{
  stepper_.setEnablePin(enable_pin_);
  stepper_.setPinsInverted(invert_direction_, false, enable_active_low_);
  stepper_.enableOutputs();
  stepper_.setCurrentPosition(0);
  enabled_ = true;
}

void SteeringStepper::set_motion(uint32_t max_speed, uint32_t accel)
{
  stepper_.setMaxSpeed(static_cast<float>(max_speed));
  stepper_.setAcceleration(static_cast<float>(accel));
}

void SteeringStepper::move_to(int32_t steps)
{
  stepper_.moveTo(steps);
}

void SteeringStepper::run()
{
  stepper_.run();
}

int32_t SteeringStepper::position()
{
  return stepper_.currentPosition();
}

void SteeringStepper::zero()
{
  stepper_.setCurrentPosition(0);
}

bool SteeringStepper::enabled() const
{
  return enabled_;
}
