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

#ifndef BICYCLE_HARDWARE__PROTOCOL_HPP_
#define BICYCLE_HARDWARE__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "bicycle_uno/protocol.hpp"  // the constants shared with the firmware

// Host side of the serial protocol: encodes requests and parses the firmware's replies.
namespace bicycle_hardware::protocol
{

// One request line, terminator included, held on the stack so the control loop never
// allocates.
class Request
{
public:
  std::string_view view() const {return {buffer_.data(), size_};}

private:
  friend class RequestBuilder;
  std::array<char, bicycle_protocol::kMaxLineLength + 1> buffer_{};
  std::size_t size_{0};
};

Request version_request();
Request limits_request(int32_t max_steps, uint32_t max_speed, uint32_t accel);
Request command_request(int32_t pwm, int32_t steering_steps);
Request query_request(uint16_t seq);
Request zero_request();

struct FirmwareVersion
{
  std::string name;
  int32_t version;
};

struct Limits
{
  int32_t max_steps;
  uint32_t max_speed;
  uint32_t accel;
};

struct State
{
  uint16_t seq;
  int32_t encoder_counts;
  int32_t pulse_period_us;
  int32_t steering_steps;
  uint8_t flags;
};

// Each parser accepts exactly its reply ("V ...", "K ...", "S ...") and rejects anything else,
// including extra or malformed fields.
std::optional<FirmwareVersion> parse_version(std::string_view line);
std::optional<Limits> parse_limits(std::string_view line);
std::optional<State> parse_state(std::string_view line);
// Error code of an "E <code>" reply.
std::optional<uint8_t> parse_error(std::string_view line);

}  // namespace bicycle_hardware::protocol

#endif  // BICYCLE_HARDWARE__PROTOCOL_HPP_
