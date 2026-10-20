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

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>

#include "bicycle_hardware/protocol.hpp"

namespace protocol = bicycle_hardware::protocol;

TEST(Protocol, EncodesRequests)
{
  EXPECT_EQ(protocol::version_request().view(), "V\n");
  EXPECT_EQ(protocol::zero_request().view(), "Z\n");
  EXPECT_EQ(protocol::query_request(7).view(), "Q 7\n");
  EXPECT_EQ(protocol::command_request(-120, 35).view(), "C -120 35\n");
  EXPECT_EQ(protocol::limits_request(200, 509, 1019).view(), "L 200 509 1019\n");
}

TEST(Protocol, EncodesExtremeValues)
{
  const auto request = protocol::limits_request(
    std::numeric_limits<int32_t>::min(), std::numeric_limits<uint32_t>::max(),
    std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(request.view(), "L -2147483648 4294967295 4294967295\n");
  EXPECT_LE(request.view().size(), bicycle_protocol::kMaxLineLength + 1u);
}

TEST(Protocol, ParsesState)
{
  const auto state = protocol::parse_state("S 12 -34 -250000 120 5");
  ASSERT_TRUE(state);
  EXPECT_EQ(state->seq, 12);
  EXPECT_EQ(state->encoder_counts, -34);
  EXPECT_EQ(state->pulse_period_us, -250000);
  EXPECT_EQ(state->steering_steps, 120);
  EXPECT_EQ(state->flags, 5);
}

// H16: the original parser turned malformed replies into zeros.
TEST(Protocol, RejectsMalformedState)
{
  for (const char * bad : {
    "", "S", "S 1 2 3 4", "S 1 2 3 4 5 6", "S 1 2 3 4 x", "S 1 2 3 4 256", "S 70000 0 0 0 0",
    "S  1 2 3 4 5", "S 1 2 3 4 5 ", "T 1 2 3 4 5", "SS 1 2 3 4 5", "S 1 2.5 3 4 5",
    "S 1 99999999999 3 4 5"})
  {
    EXPECT_FALSE(protocol::parse_state(bad)) << '"' << bad << '"';
  }
}

TEST(Protocol, ParsesVersionLimitsAndErrors)
{
  const auto version = protocol::parse_version("V bicycle_uno 1");
  ASSERT_TRUE(version);
  EXPECT_EQ(version->name, "bicycle_uno");
  EXPECT_EQ(version->version, 1);
  EXPECT_FALSE(protocol::parse_version("V bicycle_uno"));
  EXPECT_FALSE(protocol::parse_version("V bicycle_uno one"));

  const auto limits = protocol::parse_limits("K 200 509 1019");
  ASSERT_TRUE(limits);
  EXPECT_EQ(limits->max_steps, 200);
  EXPECT_EQ(limits->max_speed, 509u);
  EXPECT_EQ(limits->accel, 1019u);
  EXPECT_FALSE(protocol::parse_limits("K"));
  EXPECT_FALSE(protocol::parse_limits("K 200 -1 1019"));

  EXPECT_EQ(protocol::parse_error("E 2"), 2);
  EXPECT_FALSE(protocol::parse_error("E"));
  EXPECT_FALSE(protocol::parse_error("S 1 2 3 4 5"));
}
