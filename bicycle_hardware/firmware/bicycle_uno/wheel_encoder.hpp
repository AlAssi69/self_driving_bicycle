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

#ifndef BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__WHEEL_ENCODER_HPP_
#define BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__WHEEL_ENCODER_HPP_

#include <stdint.h>

#include "firmware_core.hpp"

// Single-channel IR slot sensor on the rear wheel, counted in an interrupt.
//
// The sensor cannot tell the direction, so each pulse takes the sign of the drive direction
// set with set_direction().
namespace wheel_encoder
{

// Attaches the interrupt. Call from setup().
void begin(uint8_t pin, uint32_t debounce_us);

// +1 or -1: sign of the following pulses.
void set_direction(int8_t direction);

// Consistent snapshot of the count and pulse period (see bicycle_protocol::kStateReply).
bicycle_firmware::EncoderReading read();

}  // namespace wheel_encoder

#endif  // BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__WHEEL_ENCODER_HPP_
