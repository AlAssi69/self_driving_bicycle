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

#include <cmath>
#include <limits>

#include "bicycle_hardware/drive_model.hpp"

using bicycle_hardware::DriveController;
using bicycle_hardware::DriveGains;
using bicycle_hardware::SteeringModel;
using bicycle_hardware::WheelEncoderModel;

// H12: the original plugin reported 0.2 per count instead of 2 pi / 8 radians.
TEST(WheelEncoderModel, CountsToRadians)
{
  const WheelEncoderModel encoder(8, 2.0);
  EXPECT_DOUBLE_EQ(encoder.to_radians(1), M_PI / 4);
  EXPECT_DOUBLE_EQ(encoder.to_radians(8), 2 * M_PI);
  EXPECT_DOUBLE_EQ(encoder.to_radians(-4), -M_PI);
}

// H13: no division by zero, and the speed decays to zero once pulses stop.
TEST(WheelEncoderModel, VelocityFromPulsePeriod)
{
  const WheelEncoderModel encoder(8, 2.0);
  EXPECT_DOUBLE_EQ(encoder.velocity(0), 0.0);
  EXPECT_DOUBLE_EQ(encoder.velocity(500000), (M_PI / 4) / 0.5);
  EXPECT_DOUBLE_EQ(encoder.velocity(-500000), -(M_PI / 4) / 0.5);
  EXPECT_DOUBLE_EQ(encoder.velocity(2000000), (M_PI / 4) / 2.0);
  EXPECT_DOUBLE_EQ(encoder.velocity(2000001), 0.0);
  EXPECT_DOUBLE_EQ(encoder.velocity(std::numeric_limits<int32_t>::min()), 0.0);
}

TEST(SteeringModel, StepsAndRadians)
{
  const double steps_per_rad = bicycle_hardware::steps_per_radian(200, 8, 1.0);
  EXPECT_NEAR(steps_per_rad, 1600 / (2 * M_PI), 1e-12);
  const SteeringModel steering(steps_per_rad);
  EXPECT_EQ(steering.to_steps(0.7854), 200);
  EXPECT_EQ(steering.to_steps(-0.7854), -200);
  EXPECT_NEAR(steering.to_radians(200), 0.7854, 1e-3);
}

namespace
{

DriveGains feedforward_only()
{
  return {25.0, 25.0, 150.0, 0.0, 0.0, 30.0};
}

}  // namespace

TEST(DriveController, FeedforwardWithDeadband)
{
  DriveController drive(feedforward_only());
  EXPECT_EQ(drive.update(1.6, 0.0, 0.01), 65);
  EXPECT_EQ(drive.update(-1.6, 0.0, 0.01), -65);
}

TEST(DriveController, StopsForZeroOrInvalidTargets)
{
  DriveController drive(feedforward_only());
  EXPECT_EQ(drive.update(0.0, 1.0, 0.01), 0);
  EXPECT_EQ(drive.update(std::nan(""), 0.0, 0.01), 0);
  EXPECT_EQ(drive.update(std::numeric_limits<double>::infinity(), 0.0, 0.01), 0);
}

TEST(DriveController, ClampsToMaxPwm)
{
  DriveController drive(feedforward_only());
  EXPECT_EQ(drive.update(100.0, 0.0, 0.01), 150);
  EXPECT_EQ(drive.update(-100.0, 0.0, 0.01), -150);
}

TEST(DriveController, NeverReversesAgainstTheTarget)
{
  DriveGains gains = feedforward_only();
  gains.kp = 100.0;
  DriveController drive(gains);
  EXPECT_EQ(drive.update(0.5, 3.0, 0.01), 0);
  EXPECT_EQ(drive.update(-0.5, -3.0, 0.01), 0);
}

TEST(DriveController, IntegralIsBoundedAndReset)
{
  DriveGains gains = feedforward_only();
  gains.ki = 10.0;
  DriveController drive(gains);
  int32_t pwm = 0;
  for (int i = 0; i < 1000; ++i) {
    pwm = drive.update(1.0, 0.0, 0.01);
  }
  EXPECT_EQ(pwm, 25 + 25 + 30);  // feedforward + deadband + integral_limit
  drive.update(0.0, 0.0, 0.01);
  EXPECT_EQ(drive.update(1.0, 1.0, 0.0), 50);
}

TEST(FrontWheel, RollsFartherWhenSteering)
{
  EXPECT_DOUBLE_EQ(bicycle_hardware::front_wheel_ratio(0.0), 1.0);
  EXPECT_NEAR(bicycle_hardware::front_wheel_ratio(0.7854), std::sqrt(2.0), 1e-4);
  EXPECT_DOUBLE_EQ(bicycle_hardware::front_wheel_ratio(M_PI / 2), 10.0);
}
