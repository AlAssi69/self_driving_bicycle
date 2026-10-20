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

#include "bicycle_hardware/protocol.hpp"

#include <charconv>
#include <limits>
#include <system_error>

namespace bicycle_hardware::protocol
{

// The longest request: the type, three 32-bit fields of up to 11 characters with their
// separators, and the terminator.
static_assert(bicycle_protocol::kMaxLineLength >= 1 + 3 * 12, "requests must fit in a line");

class RequestBuilder
{
public:
  explicit RequestBuilder(char type)
  {
    request_.buffer_[0] = type;
    request_.size_ = 1;
  }

  template<typename Integer>
  RequestBuilder & add(Integer value)
  {
    static_assert(sizeof(Integer) <= sizeof(int32_t), "fields are at most 32 bits");
    auto & buffer = request_.buffer_;
    buffer[request_.size_++] = ' ';
    const auto result = std::to_chars(
      buffer.data() + request_.size_, buffer.data() + buffer.size() - 1, value);
    request_.size_ = static_cast<std::size_t>(result.ptr - buffer.data());
    return *this;
  }

  Request build()
  {
    request_.buffer_[request_.size_++] = bicycle_protocol::kTerminator;
    return request_;
  }

private:
  Request request_;
};

Request version_request()
{
  return RequestBuilder(bicycle_protocol::kVersionRequest).build();
}

Request limits_request(int32_t max_steps, uint32_t max_speed, uint32_t accel)
{
  return RequestBuilder(bicycle_protocol::kLimitsRequest)
         .add(max_steps).add(max_speed).add(accel).build();
}

Request command_request(int32_t pwm, int32_t steering_steps)
{
  return RequestBuilder(bicycle_protocol::kCommandRequest).add(pwm).add(steering_steps).build();
}

Request query_request(uint16_t seq)
{
  return RequestBuilder(bicycle_protocol::kQueryRequest).add(seq).build();
}

Request zero_request()
{
  return RequestBuilder(bicycle_protocol::kZeroRequest).build();
}

namespace
{

// Splits a line on single spaces into exactly N fields; fails on any other shape.
template<std::size_t N>
std::optional<std::array<std::string_view, N>> split(std::string_view line)
{
  std::array<std::string_view, N> fields;
  for (std::size_t i = 0; i < N; ++i) {
    const auto space = line.find(' ');
    const bool last = i + 1 == N;
    if (last != (space == std::string_view::npos)) {
      return std::nullopt;
    }
    fields[i] = line.substr(0, space);
    if (fields[i].empty()) {
      return std::nullopt;
    }
    line = last ? std::string_view{} : line.substr(space + 1);
  }
  return fields;
}

template<typename Integer>
std::optional<Integer> to_integer(std::string_view text)
{
  Integer value{};
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    return std::nullopt;
  }
  return value;
}

bool is_type(std::string_view field, char type)
{
  return field.size() == 1 && field[0] == type;
}

}  // namespace

std::optional<FirmwareVersion> parse_version(std::string_view line)
{
  const auto fields = split<3>(line);
  if (!fields || !is_type((*fields)[0], bicycle_protocol::kVersionReply)) {
    return std::nullopt;
  }
  const auto version = to_integer<int32_t>((*fields)[2]);
  if (!version) {
    return std::nullopt;
  }
  return FirmwareVersion{std::string((*fields)[1]), *version};
}

std::optional<Limits> parse_limits(std::string_view line)
{
  const auto fields = split<4>(line);
  if (!fields || !is_type((*fields)[0], bicycle_protocol::kOkReply)) {
    return std::nullopt;
  }
  const auto max_steps = to_integer<int32_t>((*fields)[1]);
  const auto max_speed = to_integer<uint32_t>((*fields)[2]);
  const auto accel = to_integer<uint32_t>((*fields)[3]);
  if (!max_steps || !max_speed || !accel) {
    return std::nullopt;
  }
  return Limits{*max_steps, *max_speed, *accel};
}

std::optional<State> parse_state(std::string_view line)
{
  const auto fields = split<6>(line);
  if (!fields || !is_type((*fields)[0], bicycle_protocol::kStateReply)) {
    return std::nullopt;
  }
  const auto seq = to_integer<uint16_t>((*fields)[1]);
  const auto counts = to_integer<int32_t>((*fields)[2]);
  const auto period = to_integer<int32_t>((*fields)[3]);
  const auto steps = to_integer<int32_t>((*fields)[4]);
  const auto flags = to_integer<uint8_t>((*fields)[5]);
  if (!seq || !counts || !period || !steps || !flags) {
    return std::nullopt;
  }
  return State{*seq, *counts, *period, *steps, *flags};
}

std::optional<uint8_t> parse_error(std::string_view line)
{
  const auto fields = split<2>(line);
  if (!fields || !is_type((*fields)[0], bicycle_protocol::kErrorReply)) {
    return std::nullopt;
  }
  return to_integer<uint8_t>((*fields)[1]);
}

}  // namespace bicycle_hardware::protocol
