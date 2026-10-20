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

#include "bicycle_hardware/drive_model.hpp"

#include <algorithm>
#include <cmath>

namespace bicycle_hardware
{
namespace
{

constexpr double kTwoPi = 2.0 * M_PI;
// Commands below this speed (rad/s) mean "stop".
constexpr double kStopThreshold = 1e-3;
// Keeps the front-wheel ratio finite near a (physically impossible) 90 degree steering angle.
constexpr double kMinCos = 0.1;

}  // namespace

WheelEncoderModel::WheelEncoderModel(double counts_per_rev, double timeout_s)
: rad_per_count_(kTwoPi / counts_per_rev), timeout_us_(timeout_s * 1e6)
{
}

double WheelEncoderModel::to_radians(int64_t counts) const
{
  return static_cast<double>(counts) * rad_per_count_;
}

double WheelEncoderModel::velocity(int32_t pulse_period_us) const
{
  const double period = std::abs(static_cast<double>(pulse_period_us));
  if (pulse_period_us == 0 || period > timeout_us_) {
    return 0.0;
  }
  return std::copysign(rad_per_count_ * 1e6 / period, static_cast<double>(pulse_period_us));
}

SteeringModel::SteeringModel(double steps_per_rad)
: steps_per_rad_(steps_per_rad)
{
}

int32_t SteeringModel::to_steps(double angle) const
{
  return static_cast<int32_t>(std::lround(angle * steps_per_rad_));
}

double SteeringModel::to_radians(int32_t steps) const
{
  return static_cast<double>(steps) / steps_per_rad_;
}

double steps_per_radian(double steps_per_rev, double microsteps, double gear_ratio)
{
  return steps_per_rev * microsteps * gear_ratio / kTwoPi;
}

DriveController::DriveController(const DriveGains & gains)
: gains_(gains)
{
}

int32_t DriveController::update(double target, double measured, double dt)
{
  if (!std::isfinite(target) || std::abs(target) < kStopThreshold) {
    reset();
    return 0;
  }
  const double direction = std::copysign(1.0, target);
  const double error = target - measured;
  if (gains_.ki > 0.0 && dt > 0.0) {
    const double limit = gains_.integral_limit / gains_.ki;
    integral_ = std::clamp(integral_ + error * dt, -limit, limit);
  }
  double pwm = gains_.feedforward * target + direction * gains_.deadband + gains_.kp * error +
    gains_.ki * integral_;
  // Never brake by reversing: the encoder cannot see reverse speed.
  pwm = direction > 0.0 ? std::clamp(pwm, 0.0, gains_.max_pwm) :
    std::clamp(pwm, -gains_.max_pwm, 0.0);
  return static_cast<int32_t>(std::lround(pwm));
}

void DriveController::reset()
{
  integral_ = 0.0;
}

double front_wheel_ratio(double steering_angle)
{
  return 1.0 / std::max(std::cos(steering_angle), kMinCos);
}

}  // namespace bicycle_hardware
