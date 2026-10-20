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

#include "drive_motor.hpp"

#include <Arduino.h>

DriveMotor::DriveMotor(
  uint8_t forward_pwm_pin, uint8_t reverse_pwm_pin, uint8_t forward_enable_pin,
  uint8_t reverse_enable_pin)
: forward_pwm_pin_(forward_pwm_pin), reverse_pwm_pin_(reverse_pwm_pin),
  forward_enable_pin_(forward_enable_pin), reverse_enable_pin_(reverse_enable_pin)
{
}

void DriveMotor::begin()
{
  pinMode(forward_pwm_pin_, OUTPUT);
  pinMode(reverse_pwm_pin_, OUTPUT);
  set_pwm(0);
  pinMode(forward_enable_pin_, OUTPUT);
  pinMode(reverse_enable_pin_, OUTPUT);
  digitalWrite(forward_enable_pin_, HIGH);
  digitalWrite(reverse_enable_pin_, HIGH);
}

void DriveMotor::set_pwm(int32_t pwm)
{
  const int duty = static_cast<int>(constrain(pwm < 0 ? -pwm : pwm, 0, 255));
  // Release the opposite channel first so both half-bridges never drive at once.
  if (pwm >= 0) {
    analogWrite(reverse_pwm_pin_, 0);
    analogWrite(forward_pwm_pin_, duty);
  } else {
    analogWrite(forward_pwm_pin_, 0);
    analogWrite(reverse_pwm_pin_, duty);
  }
}
