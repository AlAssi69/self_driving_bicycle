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

#include "arduino_io.hpp"

#include <Arduino.h>

#include "protocol.hpp"
#include "wheel_encoder.hpp"

ArduinoIo::ArduinoIo(DriveMotor & drive, SteeringStepper & steering)
: drive_(drive), steering_(steering)
{
}

uint32_t ArduinoIo::now_ms()
{
  return millis();
}

void ArduinoIo::set_drive_pwm(int32_t pwm)
{
  // Pulses keep the last direction while the motor coasts at PWM 0.
  if (pwm != 0) {
    wheel_encoder::set_direction(pwm > 0 ? 1 : -1);
  }
  drive_.set_pwm(pwm);
}

void ArduinoIo::set_steering_target(int32_t steps)
{
  steering_.move_to(steps);
}

void ArduinoIo::set_steering_motion(uint32_t max_speed, uint32_t accel)
{
  steering_.set_motion(max_speed, accel);
}

int32_t ArduinoIo::steering_position()
{
  return steering_.position();
}

void ArduinoIo::zero_steering()
{
  steering_.zero();
}

bool ArduinoIo::steering_enabled()
{
  return steering_.enabled();
}

bicycle_firmware::EncoderReading ArduinoIo::encoder()
{
  return wheel_encoder::read();
}

void ArduinoIo::write_line(const char * line)
{
  Serial.print(line);
  Serial.print(bicycle_protocol::kTerminator);
}
