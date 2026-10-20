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

// Wiring and safety limits of the bicycle's Arduino Uno. The only file to edit for a different
// wiring; docs/hardware.md has the matching wiring table.

#ifndef BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__CONFIG_HPP_
#define BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__CONFIG_HPP_

#include <stdint.h>

#include "firmware_core.hpp"

namespace bicycle_config
{

// Rear-wheel IR slot sensor. Must be an interrupt pin (2 or 3 on the Uno).
constexpr uint8_t kEncoderPin = 2;
// The IR sensor bounces; 95 ms (the original tuning) still allows ~10 pulses/s.
constexpr uint32_t kEncoderDebounceUs = 95000UL;

// BTS7960 (IBT-2) drive motor driver. The forward channel (LPWM) is the one the original
// firmware drove forward.
constexpr uint8_t kDriveForwardPwmPin = 6;     // LPWM
constexpr uint8_t kDriveReversePwmPin = 3;     // RPWM
constexpr uint8_t kDriveForwardEnablePin = 5;   // L_EN
constexpr uint8_t kDriveReverseEnablePin = 8;   // R_EN

// Steering stepper driver (STEP/DIR/ENABLE inputs).
// CONFIRM these three pins against the bike's wiring before flashing.
constexpr uint8_t kSteeringStepPin = 9;
constexpr uint8_t kSteeringDirPin = 10;
constexpr uint8_t kSteeringEnablePin = 11;
constexpr bool kSteeringEnableActiveLow = true;
constexpr bool kSteeringInvertDirection = false;

// Hard limits. The host sends tighter ones derived from the URDF ("L" request) and can never
// exceed these. Steps assume 1600 steps/rev (200 x 8 microsteps, no gearing): 220 steps is
// about 0.86 rad, just beyond the 0.785 rad steering limit; keep them in sync with
// bicycle_hardware/config/hardware.yaml (a test checks it).
constexpr bicycle_firmware::HardLimits kHardLimits = {
  220,   // max_steering_steps
  600,   // max_steering_speed, steps/s
  1200,   // max_steering_accel, steps/s^2
  255,   // max_pwm
};

}  // namespace bicycle_config

#endif  // BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__CONFIG_HPP_
