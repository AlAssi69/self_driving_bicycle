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

// Serial protocol between the PC (bicycle_hardware plugin) and the Arduino firmware.
//
// This header is the single definition of the protocol: the firmware, the host plugin and the
// fake MCU all include it. It must stay C++11 without the C++ standard library (avr-libc).
// docs/hardware_protocol.md describes the messages.

#ifndef BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__PROTOCOL_HPP_
#define BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__PROTOCOL_HPP_

#include <stdint.h>

namespace bicycle_protocol
{

// Bump on every incompatible change; the host refuses to activate on a mismatch.
constexpr int32_t kVersion = 1;
constexpr char kFirmwareName[] = "bicycle_uno";

constexpr uint32_t kBaudRate = 115200;
// Longest line in either direction, without the terminator.
constexpr uint8_t kMaxLineLength = 63;
constexpr char kTerminator = '\n';

// The firmware stops the drive motor when no command arrived for this long.
constexpr uint32_t kCommandTimeoutMs = 300;

constexpr int32_t kMaxPwm = 255;

// Requests (PC -> MCU).
constexpr char kVersionRequest = 'V';   // "V"                            -> "V <name> <version>"
constexpr char kLimitsRequest = 'L';   // "L <max_steps> <steps_s> <steps_s2>" -> "K" | "E <code>"
constexpr char kCommandRequest = 'C';   // "C <pwm> <steering_steps>"     (no reply)
constexpr char kQueryRequest = 'Q';    // "Q <seq>"                      -> state reply
constexpr char kZeroRequest = 'Z';     // "Z"                            -> "K"

// Replies (MCU -> PC).
constexpr char kVersionReply = 'V';
// "S <seq> <encoder_counts> <pulse_period_us> <steering_steps> <flags>"
//   encoder_counts   signed pulse count since boot (sign = drive direction at the pulse)
//   pulse_period_us  signed time between the last two pulses, or since the last pulse when that
//                    is longer (so the speed decays when the wheel stops); 0 = no pulse yet
constexpr char kStateReply = 'S';
constexpr char kOkReply = 'K';
constexpr char kErrorReply = 'E';

// Flags in the state reply.
constexpr uint8_t kFlagWatchdog = 1u << 0;        // drive stopped: no command for the timeout
constexpr uint8_t kFlagSteeringClamped = 1u << 1;   // last steering target was beyond the limits
constexpr uint8_t kFlagStepperEnabled = 1u << 2;
constexpr uint8_t kFlagCommandError = 1u << 3;    // a request was rejected since the last query

// Error codes in the error reply.
constexpr uint8_t kErrorUnknownRequest = 1;
constexpr uint8_t kErrorBadArguments = 2;
constexpr uint8_t kErrorLineTooLong = 3;

}  // namespace bicycle_protocol

#endif  // BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__PROTOCOL_HPP_
