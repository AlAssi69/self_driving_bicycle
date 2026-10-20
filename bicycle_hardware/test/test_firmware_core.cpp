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

// The Arduino firmware's request handling, limits and watchdog, run on the host.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "bicycle_uno/firmware_core.hpp"
#include "bicycle_uno/protocol.hpp"

namespace
{

class RecordingIo : public bicycle_firmware::FirmwareIo
{
public:
  uint32_t now_ms() override {return now;}
  void set_drive_pwm(int32_t value) override {pwm = value;}
  void set_steering_target(int32_t steps) override {steering_target = steps;}
  void set_steering_motion(uint32_t speed, uint32_t accel) override
  {
    max_speed = speed;
    max_accel = accel;
  }
  int32_t steering_position() override {return steering_position_steps;}
  void zero_steering() override {zeroed = true;}
  bool steering_enabled() override {return true;}
  bicycle_firmware::EncoderReading encoder() override {return {counts, period_us};}
  void write_line(const char * line) override {replies.emplace_back(line);}

  uint32_t now = 0;
  int32_t pwm = 99;
  int32_t steering_target = 0;
  uint32_t max_speed = 0;
  uint32_t max_accel = 0;
  int32_t steering_position_steps = 0;
  bool zeroed = false;
  int32_t counts = 0;
  int32_t period_us = 0;
  std::vector<std::string> replies;
};

constexpr bicycle_firmware::HardLimits kLimits{200, 500, 1000, 180};

class FirmwareCoreTest : public ::testing::Test
{
protected:
  FirmwareCoreTest()
  : core(io, kLimits)
  {
    core.begin();
  }

  void send(const std::string & line)
  {
    for (const char c : line) {
      core.feed(c);
    }
    core.feed('\n');
  }

  std::string last_reply() const
  {
    return io.replies.empty() ? "" : io.replies.back();
  }

  RecordingIo io;
  bicycle_firmware::FirmwareCore core;
};

TEST_F(FirmwareCoreTest, StartsStoppedWithHardLimits)
{
  EXPECT_EQ(io.pwm, 0);
  EXPECT_EQ(io.max_speed, 500u);
  EXPECT_EQ(io.max_accel, 1000u);
  EXPECT_TRUE(core.watchdog_tripped());
}

TEST_F(FirmwareCoreTest, ReportsVersion)
{
  send("V");
  EXPECT_EQ(last_reply(), "V bicycle_uno " + std::to_string(bicycle_protocol::kVersion));
}

TEST_F(FirmwareCoreTest, CommandSetsDriveAndSteeringWithoutReply)
{
  send("C -120 150");
  EXPECT_EQ(io.pwm, -120);
  EXPECT_EQ(io.steering_target, 150);
  EXPECT_TRUE(io.replies.empty());
  EXPECT_FALSE(core.watchdog_tripped());
}

TEST_F(FirmwareCoreTest, ClampsToHardLimitsAndFlagsSteering)
{
  send("C 255 -900");
  EXPECT_EQ(io.pwm, 180);
  EXPECT_EQ(io.steering_target, -200);
  send("Q 1");
  EXPECT_EQ(last_reply(), "S 1 0 0 0 6");  // steering clamped + stepper enabled
  send("Q 2");
  EXPECT_EQ(last_reply(), "S 2 0 0 0 4");  // sticky flags are cleared once reported
}

TEST_F(FirmwareCoreTest, RejectsOutOfRangeAndMalformedCommands)
{
  send("C 10 20");
  for (const char * bad : {"C 256 0", "C -256 0", "C 10", "C 10 20 30", "C x 20", "C 10 2e3",
      "C 99999999999 0"})
  {
    send(bad);
    EXPECT_EQ(io.pwm, 10) << bad;
    EXPECT_EQ(io.steering_target, 20) << bad;
  }
  send("Q 3");
  EXPECT_EQ(last_reply(), "S 3 0 0 0 12");  // command error + stepper enabled
}

TEST_F(FirmwareCoreTest, QueryEchoesSequenceAndState)
{
  io.counts = -17;
  io.period_us = -250000;
  io.steering_position_steps = 42;
  send("C 50 42");
  send("Q 65535");
  EXPECT_EQ(last_reply(), "S 65535 -17 -250000 42 4");
}

TEST_F(FirmwareCoreTest, RejectsBadQueries)
{
  for (const char * bad : {"Q", "Q -1", "Q 65536", "Q 1 2"}) {
    send(bad);
    EXPECT_EQ(last_reply(), "E 2") << bad;
  }
}

TEST_F(FirmwareCoreTest, LimitsAreCappedByHardLimits)
{
  send("L 100 300 600");
  EXPECT_EQ(last_reply(), "K 100 300 600");
  EXPECT_EQ(io.max_speed, 300u);
  send("C 0 150");
  EXPECT_EQ(io.steering_target, 100);

  send("L 900 900 9000");
  EXPECT_EQ(last_reply(), "K 200 500 1000");

  for (const char * bad : {"L 0 300 600", "L 100 -1 600", "L 100 300", "L a b c"}) {
    send(bad);
    EXPECT_EQ(last_reply(), "E 2") << bad;
  }
}

TEST_F(FirmwareCoreTest, ZeroResetsSteering)
{
  send("Z");
  EXPECT_TRUE(io.zeroed);
  EXPECT_EQ(last_reply(), "K");
}

// H19: the original firmware dereferenced a null pointer for lines without a space.
TEST_F(FirmwareCoreTest, IgnoresEmptyLinesAndRejectsUnknownRequests)
{
  send("");
  send("   ");
  EXPECT_TRUE(io.replies.empty());
  for (const char * bad : {"m", "e", "X 1", "CC 1 2", "C1 2"}) {
    send(bad);
    EXPECT_EQ(last_reply(), "E 1") << bad;
  }
}

TEST_F(FirmwareCoreTest, ToleratesCrLf)
{
  for (const char c : std::string("V\r")) {
    core.feed(c);
  }
  core.feed('\n');
  EXPECT_EQ(last_reply(), "V bicycle_uno " + std::to_string(bicycle_protocol::kVersion));
}

TEST_F(FirmwareCoreTest, RejectsOverlongLinesAndRecovers)
{
  send("C " + std::string(200, '1'));
  EXPECT_EQ(last_reply(), "E 3");
  send("C 30 0");
  EXPECT_EQ(io.pwm, 30);
}

TEST_F(FirmwareCoreTest, RejectsTooManyTokens)
{
  send("Q 1 2 3 4 5 6 7");
  EXPECT_EQ(last_reply(), "E 2");
}

// H22: the drive stops when the host goes silent, and resumes with the next command.
TEST_F(FirmwareCoreTest, WatchdogStopsTheDrive)
{
  io.now = 1000;
  send("C 100 0");
  io.now += bicycle_protocol::kCommandTimeoutMs;
  core.tick();
  EXPECT_EQ(io.pwm, 100);
  io.now += 1;
  core.tick();
  EXPECT_EQ(io.pwm, 0);
  EXPECT_TRUE(core.watchdog_tripped());
  send("Q 9");
  EXPECT_EQ(last_reply(), "S 9 0 0 0 5");  // watchdog + stepper enabled

  send("C 80 0");
  EXPECT_EQ(io.pwm, 80);
  EXPECT_FALSE(core.watchdog_tripped());
}

TEST_F(FirmwareCoreTest, WatchdogHandlesClockWrap)
{
  io.now = 0xFFFFFF00u;
  send("C 100 0");
  io.now = 0x00000010u;  // 272 ms later
  core.tick();
  EXPECT_EQ(io.pwm, 100);
  io.now = 0x00000100u;  // 512 ms later
  core.tick();
  EXPECT_EQ(io.pwm, 0);
}

}  // namespace
