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

#ifndef BICYCLE_HARDWARE__FAKE_MCU_HPP_
#define BICYCLE_HARDWARE__FAKE_MCU_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace bicycle_hardware
{

// A stand-in for the bicycle's Arduino: runs the real firmware logic (firmware_core.hpp, with
// the config.hpp hard limits) against a simulated drive motor, wheel encoder and stepper,
// behind a pseudo-terminal that the plugin opens like the real serial port.
class FakeMcu
{
public:
  struct Options
  {
    // Simulated hardware; the defaults match bicycle_hardware/config/hardware.yaml, so the
    // plugin's feedforward drives the wheel at the commanded speed.
    double counts_per_rev = 8.0;
    double pwm_deadband = 25.0;     // PWM below which the wheel does not turn
    double rad_s_per_pwm = 0.04;    // wheel speed per PWM above the deadband
  };

  struct Snapshot
  {
    int32_t pwm;
    int32_t steering_target;
    int32_t steering_position;
    int32_t encoder_counts;
    bool watchdog;
    uint64_t commands;  // accepted "C" requests
  };

  // Opens the pseudo-terminal; throws std::system_error on failure.
  explicit FakeMcu(const Options & options);
  FakeMcu();
  ~FakeMcu();
  FakeMcu(const FakeMcu &) = delete;
  FakeMcu & operator=(const FakeMcu &) = delete;

  // Path the plugin opens (the pseudo-terminal's slave side).
  const std::string & device() const {return device_;}

  void start();
  // Stops the firmware and closes the terminal, like unplugging the Arduino.
  void stop();
  Snapshot snapshot() const;

private:
  class Simulation;

  void run();

  Options options_;
  int master_fd_{-1};
  int slave_fd_{-1};  // kept open so the master never sees a hang-up between plugin sessions
  std::string device_;
  std::unique_ptr<Simulation> simulation_;
  mutable std::mutex mutex_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace bicycle_hardware

#endif  // BICYCLE_HARDWARE__FAKE_MCU_HPP_
