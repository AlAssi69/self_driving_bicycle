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

#include "bicycle_hardware/fake_mcu.hpp"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <string>
#include <system_error>

#include "bicycle_uno/config.hpp"
#include "bicycle_uno/firmware_core.hpp"

namespace bicycle_hardware
{
namespace
{

using Clock = std::chrono::steady_clock;

[[noreturn]] void throw_errno(const std::string & what)
{
  throw std::system_error(errno, std::generic_category(), what);
}

}  // namespace

class FakeMcu::Simulation : public bicycle_firmware::FirmwareIo
{
public:
  Simulation(const Options & options, int fd)
  : options_(options), fd_(fd), start_(Clock::now()), last_step_(start_),
    core_(*this, bicycle_config::kHardLimits)
  {
    core_.begin();
  }

  void feed(const char * data, std::size_t size)
  {
    for (std::size_t i = 0; i < size; ++i) {
      core_.feed(data[i]);
    }
  }

  // Advances the motor, encoder and stepper to now, then runs the watchdog.
  void step()
  {
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - last_step_).count();
    last_step_ = now;

    const double magnitude = std::max(0.0, std::abs(pwm_) - options_.pwm_deadband);
    wheel_velocity_ = std::copysign(magnitude * options_.rad_s_per_pwm, pwm_);
    wheel_angle_ += wheel_velocity_ * dt;
    const auto counts = static_cast<int32_t>(
      std::floor(wheel_angle_ / (2.0 * M_PI) * options_.counts_per_rev));
    if (counts != counts_) {
      const uint32_t now_us = micros();
      interval_us_ = seen_pulse_ ? now_us - last_pulse_us_ : 0;
      last_pulse_us_ = now_us;
      seen_pulse_ = true;
      direction_ = counts > counts_ ? 1 : -1;
      counts_ = counts;
    }

    steering_budget_ += max_speed_ * dt;
    const int32_t error = steering_target_ - steering_position_;
    const auto max_steps = static_cast<int32_t>(steering_budget_);
    const int32_t move = std::clamp(error, -max_steps, max_steps);
    steering_position_ += move;
    steering_budget_ = error == 0 ? 0.0 : steering_budget_ - std::abs(move);

    core_.tick();
  }

  Snapshot snapshot() const
  {
    return {pwm_, steering_target_, steering_position_, counts_, core_.watchdog_tripped(),
      commands_};
  }

  // FirmwareIo
  uint32_t now_ms() override
  {
    return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start_).count());
  }

  void set_drive_pwm(int32_t pwm) override {pwm_ = pwm;}
  // Only an accepted "C" request sets the steering target.
  void set_steering_target(int32_t steps) override
  {
    steering_target_ = steps;
    ++commands_;
  }
  void set_steering_motion(uint32_t max_speed, uint32_t) override {max_speed_ = max_speed;}
  int32_t steering_position() override {return steering_position_;}
  void zero_steering() override {steering_position_ = steering_target_ = 0;}
  bool steering_enabled() override {return true;}

  bicycle_firmware::EncoderReading encoder() override
  {
    // Same period rule as the firmware's wheel_encoder.cpp.
    bicycle_firmware::EncoderReading reading{counts_, 0};
    if (interval_us_ != 0) {
      const uint32_t since = micros() - last_pulse_us_;
      const uint32_t period = std::min<uint32_t>(std::max(interval_us_, since), 2000000000U);
      reading.period_us = direction_ * static_cast<int32_t>(period);
    }
    return reading;
  }

  void write_line(const char * line) override
  {
    std::string data(line);
    data += bicycle_protocol::kTerminator;
    // A full terminal buffer means nobody reads: drop the reply like a real UART would.
    const ssize_t written = ::write(fd_, data.data(), data.size());
    static_cast<void>(written);
  }

private:
  uint32_t micros() const
  {
    return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_).count());
  }

  Options options_;
  int fd_;
  Clock::time_point start_;
  Clock::time_point last_step_;
  bicycle_firmware::FirmwareCore core_;

  int32_t pwm_{0};
  double wheel_velocity_{0.0};
  double wheel_angle_{0.0};
  int32_t counts_{0};
  int32_t direction_{1};
  bool seen_pulse_{false};
  uint32_t last_pulse_us_{0};
  uint32_t interval_us_{0};

  uint32_t max_speed_{0};
  double steering_budget_{0.0};
  int32_t steering_target_{0};
  int32_t steering_position_{0};

  uint64_t commands_{0};
};

FakeMcu::FakeMcu()
: FakeMcu(Options{})
{
}

FakeMcu::FakeMcu(const Options & options)
: options_(options)
{
  master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (master_fd_ < 0) {
    throw_errno("posix_openpt failed");
  }
  char name[128];
  if (::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0 ||
    ::ptsname_r(master_fd_, name, sizeof(name)) != 0)
  {
    const int error = errno;
    ::close(master_fd_);
    throw std::system_error(error, std::generic_category(), "cannot set up the pseudo-terminal");
  }
  device_ = name;
  slave_fd_ = ::open(name, O_RDWR | O_NOCTTY | O_CLOEXEC);
  termios tty{};
  if (slave_fd_ < 0 || ::tcgetattr(slave_fd_, &tty) != 0) {
    const int error = errno;
    ::close(master_fd_);
    throw std::system_error(error, std::generic_category(), "cannot open " + device_);
  }
  // Raw from the start: the default line discipline would echo replies back as requests.
  ::cfmakeraw(&tty);
  ::tcsetattr(slave_fd_, TCSANOW, &tty);
  simulation_ = std::make_unique<Simulation>(options_, master_fd_);
}

FakeMcu::~FakeMcu()
{
  stop();
  if (slave_fd_ >= 0) {
    ::close(slave_fd_);
  }
}

void FakeMcu::start()
{
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread([this] {run();});
}

void FakeMcu::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (master_fd_ >= 0) {
    ::close(master_fd_);
    master_fd_ = -1;
  }
  if (slave_fd_ >= 0) {
    ::close(slave_fd_);
    slave_fd_ = -1;
  }
}

FakeMcu::Snapshot FakeMcu::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return simulation_->snapshot();
}

void FakeMcu::run()
{
  char buffer[256];
  while (running_) {
    pollfd pfd{master_fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, 1);
    std::lock_guard<std::mutex> lock(mutex_);
    if (ready > 0 && (pfd.revents & POLLIN)) {
      const ssize_t received = ::read(master_fd_, buffer, sizeof(buffer));
      if (received > 0) {
        simulation_->feed(buffer, static_cast<std::size_t>(received));
      }
    }
    simulation_->step();
  }
}

}  // namespace bicycle_hardware
