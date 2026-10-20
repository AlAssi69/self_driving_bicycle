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

// Request handling, limits and watchdog of the bicycle firmware, independent of Arduino.
//
// The sketch (bicycle_uno.ino) implements FirmwareIo with the real pins; the host fake MCU and
// the unit tests implement it with a simulation. Keep this header C++11 without the C++ standard
// library, heap allocation or Arduino macro names (min, max, abs, ...).

#ifndef BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__FIRMWARE_CORE_HPP_
#define BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__FIRMWARE_CORE_HPP_

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "protocol.hpp"

namespace bicycle_firmware
{

struct EncoderReading
{
  int32_t counts;     // signed pulse count since boot
  int32_t period_us;   // see bicycle_protocol::kStateReply
};

// Upper bounds the host cannot exceed (config.hpp on the robot).
struct HardLimits
{
  int32_t max_steering_steps;      // |steering target| in steps
  uint32_t max_steering_speed;     // steps/s
  uint32_t max_steering_accel;     // steps/s^2
  int32_t max_pwm;                 // |drive PWM|, <= bicycle_protocol::kMaxPwm
};

// Hardware seen by the core.
class FirmwareIo
{
public:
  virtual ~FirmwareIo() = default;
  virtual uint32_t now_ms() = 0;
  virtual void set_drive_pwm(int32_t pwm) = 0;
  virtual void set_steering_target(int32_t steps) = 0;
  virtual void set_steering_motion(uint32_t max_speed, uint32_t accel) = 0;
  virtual int32_t steering_position() = 0;
  virtual void zero_steering() = 0;
  virtual bool steering_enabled() = 0;
  virtual EncoderReading encoder() = 0;
  // Writes one line; the terminator is added by the implementation.
  virtual void write_line(const char * line) = 0;
};

class FirmwareCore
{
public:
  FirmwareCore(FirmwareIo & io, const HardLimits & hard_limits)
  : io_(io), hard_(hard_limits), max_steps_(hard_limits.max_steering_steps), length_(0),
    overflow_(false), last_command_ms_(0), watchdog_(true), sticky_flags_(0)
  {
  }

  // Puts the hardware in its safe start state. Call once from setup().
  void begin()
  {
    io_.set_drive_pwm(0);
    io_.set_steering_motion(hard_.max_steering_speed, hard_.max_steering_accel);
    watchdog_ = true;
  }

  // Feeds one received byte.
  void feed(char c)
  {
    if (c == bicycle_protocol::kTerminator) {
      if (overflow_) {
        reply_error(bicycle_protocol::kErrorLineTooLong);
      } else {
        line_[length_] = '\0';
        handle_line();
      }
      length_ = 0;
      overflow_ = false;
    } else if (c == '\r') {
      // Tolerate CRLF terminals.
    } else if (length_ < bicycle_protocol::kMaxLineLength) {
      line_[length_++] = c;
    } else {
      overflow_ = true;
    }
  }

  // Stops the drive when the host went silent. Call every loop.
  void tick()
  {
    if (!watchdog_ && io_.now_ms() - last_command_ms_ > bicycle_protocol::kCommandTimeoutMs) {
      io_.set_drive_pwm(0);
      watchdog_ = true;
    }
  }

  bool watchdog_tripped() const {return watchdog_;}

private:
  static constexpr uint8_t kMaxTokens = 5;

  void handle_line()
  {
    char * tokens[kMaxTokens];
    const uint8_t count = tokenize(line_, tokens);
    if (count == 0) {
      return;
    }
    if (tokens[0][1] != '\0') {
      reply_error(bicycle_protocol::kErrorUnknownRequest);
      return;
    }
    switch (tokens[0][0]) {
      case bicycle_protocol::kVersionRequest:
        handle_version(count);
        break;
      case bicycle_protocol::kLimitsRequest:
        handle_limits(tokens, count);
        break;
      case bicycle_protocol::kCommandRequest:
        handle_command(tokens, count);
        break;
      case bicycle_protocol::kQueryRequest:
        handle_query(tokens, count);
        break;
      case bicycle_protocol::kZeroRequest:
        handle_zero(count);
        break;
      default:
        reply_error(bicycle_protocol::kErrorUnknownRequest);
    }
  }

  void handle_version(uint8_t count)
  {
    if (count != 1) {
      reply_error(bicycle_protocol::kErrorBadArguments);
      return;
    }
    char reply[bicycle_protocol::kMaxLineLength + 1];
    snprintf(
      reply, sizeof(reply), "%c %s %ld", bicycle_protocol::kVersionReply,
      bicycle_protocol::kFirmwareName, static_cast<long>(bicycle_protocol::kVersion));      // NOLINT
    io_.write_line(reply);
  }

  // "L <max_steps> <steps_s> <steps_s2>": tightens the steering limits; replies with the values
  // in effect after capping them at the hard limits.
  void handle_limits(char ** tokens, uint8_t count)
  {
    long max_steps = 0;   // NOLINT(runtime/int): strtol type
    long speed = 0;   // NOLINT(runtime/int)
    long accel = 0;   // NOLINT(runtime/int)
    if (count != 4 || !parse_long(tokens[1], max_steps) || !parse_long(tokens[2], speed) ||
      !parse_long(tokens[3], accel) || max_steps <= 0 || speed <= 0 || accel <= 0)
    {
      reply_error(bicycle_protocol::kErrorBadArguments);
      return;
    }
    max_steps_ = cap(max_steps, hard_.max_steering_steps);
    const uint32_t effective_speed = static_cast<uint32_t>(cap(speed,
        hard_.max_steering_speed));
    const uint32_t effective_accel = static_cast<uint32_t>(cap(accel,
        hard_.max_steering_accel));
    io_.set_steering_motion(effective_speed, effective_accel);

    char reply[bicycle_protocol::kMaxLineLength + 1];
    snprintf(
      reply, sizeof(reply), "%c %ld %lu %lu", bicycle_protocol::kOkReply,
      static_cast<long>(max_steps_), static_cast<unsigned long>(effective_speed),          // NOLINT
      static_cast<unsigned long>(effective_accel));      // NOLINT
    io_.write_line(reply);
  }

  // "C <pwm> <steering_steps>": never replies, so the host does not wait for it.
  void handle_command(char ** tokens, uint8_t count)
  {
    long pwm = 0;   // NOLINT(runtime/int)
    long steps = 0;   // NOLINT(runtime/int)
    if (count != 3 || !parse_long(tokens[1], pwm) || !parse_long(tokens[2], steps) ||
      pwm<-bicycle_protocol::kMaxPwm || pwm> bicycle_protocol::kMaxPwm)
    {
      sticky_flags_ |= bicycle_protocol::kFlagCommandError;
      return;
    }
    if (steps > max_steps_ || steps < -max_steps_) {
      sticky_flags_ |= bicycle_protocol::kFlagSteeringClamped;
      steps = steps > 0 ? max_steps_ : -max_steps_;
    }
    if (pwm > hard_.max_pwm) {
      pwm = hard_.max_pwm;
    } else if (pwm < -hard_.max_pwm) {
      pwm = -hard_.max_pwm;
    }
    io_.set_drive_pwm(static_cast<int32_t>(pwm));
    io_.set_steering_target(static_cast<int32_t>(steps));
    last_command_ms_ = io_.now_ms();
    watchdog_ = false;
  }

  // "Q <seq>": the sequence number is echoed so the host can drop late replies.
  void handle_query(char ** tokens, uint8_t count)
  {
    long seq = 0;   // NOLINT(runtime/int)
    if (count != 2 || !parse_long(tokens[1], seq) || seq < 0 || seq > 65535) {
      reply_error(bicycle_protocol::kErrorBadArguments);
      return;
    }
    const EncoderReading encoder = io_.encoder();
    uint8_t flags = sticky_flags_;
    if (watchdog_) {
      flags |= bicycle_protocol::kFlagWatchdog;
    }
    if (io_.steering_enabled()) {
      flags |= bicycle_protocol::kFlagStepperEnabled;
    }
    sticky_flags_ = 0;

    char reply[bicycle_protocol::kMaxLineLength + 1];
    snprintf(
      reply, sizeof(reply), "%c %ld %ld %ld %ld %u", bicycle_protocol::kStateReply,
      seq, static_cast<long>(encoder.counts), static_cast<long>(encoder.period_us),          // NOLINT
      static_cast<long>(io_.steering_position()), static_cast<unsigned>(flags));          // NOLINT
    io_.write_line(reply);
  }

  void handle_zero(uint8_t count)
  {
    if (count != 1) {
      reply_error(bicycle_protocol::kErrorBadArguments);
      return;
    }
    io_.zero_steering();
    char reply[2] = {bicycle_protocol::kOkReply, '\0'};
    io_.write_line(reply);
  }

  void reply_error(uint8_t code)
  {
    char reply[8];
    snprintf(
      reply, sizeof(reply), "%c %u", bicycle_protocol::kErrorReply,
        static_cast<unsigned>(code));
    io_.write_line(reply);
  }

  // Splits the line in place on spaces; returns the number of tokens, or kMaxTokens + 1 when
  // there are too many (rejected by every handler's count check).
  static uint8_t tokenize(char * line, char ** tokens)
  {
    uint8_t count = 0;
    char * cursor = line;
    while (*cursor != '\0') {
      while (*cursor == ' ') {
        *cursor++ = '\0';
      }
      if (*cursor == '\0') {
        break;
      }
      if (count == kMaxTokens) {
        return kMaxTokens + 1;
      }
      tokens[count++] = cursor;
      while (*cursor != ' ' && *cursor != '\0') {
        ++cursor;
      }
    }
    return count;
  }

  // Accepts only a complete decimal integer that fits in 32 bits.
  static bool parse_long(const char * text, long & value)   // NOLINT(runtime/int)
  {
    char * end = nullptr;
    errno = 0;
    const long parsed = strtol(text, &end, 10);   // NOLINT(runtime/int)
    if (end == text || *end != '\0' || errno == ERANGE || parsed > 2147483647L ||
      parsed < -2147483647L)
    {
      return false;
    }
    value = parsed;
    return true;
  }

  static int32_t cap(long value, long limit)   // NOLINT(runtime/int)
  {
    return static_cast<int32_t>(value < limit ? value : limit);
  }

  FirmwareIo & io_;
  const HardLimits hard_;
  int32_t max_steps_;
  char line_[bicycle_protocol::kMaxLineLength + 1];
  uint8_t length_;
  bool overflow_;
  uint32_t last_command_ms_;
  bool watchdog_;
  uint8_t sticky_flags_;
};

}  // namespace bicycle_firmware

#endif  // BICYCLE_HARDWARE__FIRMWARE__BICYCLE_UNO__FIRMWARE_CORE_HPP_
