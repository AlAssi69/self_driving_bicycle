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

#ifndef BICYCLE_HARDWARE__DRIVE_MODEL_HPP_
#define BICYCLE_HARDWARE__DRIVE_MODEL_HPP_

#include <cstdint>

// Unit conversions and the drive speed law, free of ROS and I/O so they can be unit-tested.
namespace bicycle_hardware
{

// Rear-wheel encoder: pulse counts and pulse periods to radians and rad/s.
class WheelEncoderModel
{
public:
  // counts_per_rev > 0; timeout_s: pulse periods longer than this mean "stopped".
  WheelEncoderModel(double counts_per_rev, double timeout_s);

  double to_radians(int64_t counts) const;
  // Signed pulse period (bicycle_protocol::kStateReply) to wheel speed; 0 when no pulse has
  // been measured or the last one is older than the timeout.
  double velocity(int32_t pulse_period_us) const;

private:
  double rad_per_count_;
  double timeout_us_;
};

// Steering stepper: AccelStepper steps <-> steering angle.
class SteeringModel
{
public:
  explicit SteeringModel(double steps_per_rad);

  int32_t to_steps(double angle) const;
  double to_radians(int32_t steps) const;
  double steps_per_rad() const {return steps_per_rad_;}

private:
  double steps_per_rad_;
};

double steps_per_radian(double steps_per_rev, double microsteps, double gear_ratio);

struct DriveGains
{
  double feedforward;     // PWM per rad/s of wheel speed
  double deadband;        // PWM added in the direction of motion to overcome static friction
  double max_pwm;         // |PWM| cap
  double kp;              // PWM per rad/s of speed error
  double ki;              // PWM per rad of integrated speed error
  double integral_limit;  // |PWM| cap of the integral term
};

// Wheel speed command (rad/s) to signed PWM: feedforward + deadband, optionally corrected by
// a PI loop on the measured speed. The output never opposes the commanded direction (the
// single-channel encoder cannot measure reverse speed).
class DriveController
{
public:
  explicit DriveController(const DriveGains & gains);

  int32_t update(double target, double measured, double dt);
  void reset();

private:
  DriveGains gains_;
  double integral_{0.0};
};

// The passive front wheel rolls 1/cos(steering) times as far as the rear wheel.
double front_wheel_ratio(double steering_angle);

}  // namespace bicycle_hardware

#endif  // BICYCLE_HARDWARE__DRIVE_MODEL_HPP_
