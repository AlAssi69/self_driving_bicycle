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

// fake_mcu: a simulated bicycle Arduino on a pseudo-terminal, to run the robot stack without
// the bike:
//   ros2 run bicycle_hardware fake_mcu --link /tmp/bicycle_fake_mcu
//   ros2 launch bicycle_bringup robot_bringup.launch.py device:=/tmp/bicycle_fake_mcu ...
// Prints the drive and steering commands it receives once per second while they change.

#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "bicycle_hardware/fake_mcu.hpp"

namespace
{

std::atomic<bool> g_stop{false};

void on_signal(int signal_number)
{
  static_cast<void>(signal_number);
  g_stop = true;
}

void usage()
{
  std::cerr << "usage: fake_mcu [--link PATH] [--counts-per-rev N]\n"
    "  --link PATH         also make PATH a symlink to the pseudo-terminal\n"
    "  --counts-per-rev N  encoder pulses per rear-wheel revolution (default 8)\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string link;
  bicycle_hardware::FakeMcu::Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ros-args") {
      break;  // started by a launch file
    } else if (arg == "--link" && i + 1 < argc) {
      link = argv[++i];
    } else if (arg == "--counts-per-rev" && i + 1 < argc) {
      options.counts_per_rev = std::stod(argv[++i]);
    } else {
      usage();
      return 2;
    }
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  try {
    bicycle_hardware::FakeMcu mcu(options);
    if (!link.empty()) {
      ::unlink(link.c_str());
      if (::symlink(mcu.device().c_str(), link.c_str()) != 0) {
        std::cerr << "fake_mcu: cannot create " << link << ": " << std::strerror(errno) << "\n";
        return 1;
      }
    }
    mcu.start();
    std::cout << "fake_mcu: listening on " << mcu.device() <<
      (link.empty() ? "" : " (" + link + ")") << std::endl;

    bicycle_hardware::FakeMcu::Snapshot last{};
    while (!g_stop) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const auto now = mcu.snapshot();
      if (now.pwm != last.pwm || now.steering_target != last.steering_target ||
        now.steering_position != last.steering_position ||
        now.encoder_counts != last.encoder_counts || now.watchdog != last.watchdog)
      {
        std::cout << "fake_mcu: pwm " << now.pwm << " steering " << now.steering_position <<
          "/" << now.steering_target << " counts " << now.encoder_counts <<
          (now.watchdog ? " watchdog" : "") << std::endl;
      }
      last = now;
    }
    mcu.stop();
    if (!link.empty()) {
      ::unlink(link.c_str());
    }
  } catch (const std::exception & e) {
    std::cerr << "fake_mcu: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
