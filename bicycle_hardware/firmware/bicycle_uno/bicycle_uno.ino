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

// Arduino Uno firmware of the self-driving bicycle.
//
// Drives the rear DC motor (BTS7960) and the steering stepper, counts the rear-wheel IR
// encoder, and talks to the bicycle_hardware ros2_control plugin over USB serial
// (protocol.hpp, docs/hardware_protocol.md). It stops the drive motor when the PC stops
// sending commands. Wiring and limits: config.hpp. Requires the AccelStepper library.

#include "arduino_io.hpp"
#include "config.hpp"
#include "drive_motor.hpp"
#include "firmware_core.hpp"
#include "protocol.hpp"
#include "steering_stepper.hpp"
#include "wheel_encoder.hpp"

DriveMotor drive(
  bicycle_config::kDriveForwardPwmPin, bicycle_config::kDriveReversePwmPin,
  bicycle_config::kDriveForwardEnablePin, bicycle_config::kDriveReverseEnablePin);
SteeringStepper steering(
  bicycle_config::kSteeringStepPin, bicycle_config::kSteeringDirPin,
  bicycle_config::kSteeringEnablePin, bicycle_config::kSteeringEnableActiveLow,
  bicycle_config::kSteeringInvertDirection);
ArduinoIo io(drive, steering);
bicycle_firmware::FirmwareCore core(io, bicycle_config::kHardLimits);

void setup()
{
  // Hardware first, so the motor is stopped before any request can arrive.
  drive.begin();
  steering.begin();
  wheel_encoder::begin(bicycle_config::kEncoderPin, bicycle_config::kEncoderDebounceUs);
  core.begin();
  Serial.begin(bicycle_protocol::kBaudRate);
}

void loop()
{
  while (Serial.available() > 0) {
    core.feed(static_cast<char>(Serial.read()));
  }
  core.tick();
  steering.run();
}
