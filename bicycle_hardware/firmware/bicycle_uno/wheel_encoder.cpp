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

#include "wheel_encoder.hpp"

#include <Arduino.h>

namespace wheel_encoder
{
namespace
{

// Shared with the interrupt: volatile, and read with interrupts off (multi-byte on AVR).
volatile int32_t counts = 0;
volatile uint32_t last_pulse_us = 0;
volatile uint32_t interval_us = 0;  // between the last two pulses; 0 until two were seen
volatile int8_t direction = 1;
volatile int8_t last_pulse_direction = 1;
volatile bool seen_pulse = false;
uint32_t debounce = 0;

void on_pulse()
{
  const uint32_t now = micros();
  if (seen_pulse && now - last_pulse_us < debounce) {
    return;
  }
  interval_us = seen_pulse ? now - last_pulse_us : 0;
  last_pulse_us = now;
  seen_pulse = true;
  last_pulse_direction = direction;
  counts += direction;
}

}  // namespace

void begin(uint8_t pin, uint32_t debounce_us)
{
  debounce = debounce_us;
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), on_pulse, RISING);
}

void set_direction(int8_t new_direction)
{
  direction = new_direction < 0 ? -1 : 1;
}

bicycle_firmware::EncoderReading read()
{
  noInterrupts();
  const int32_t snapshot_counts = counts;
  const uint32_t snapshot_last = last_pulse_us;
  const uint32_t snapshot_interval = interval_us;
  const int8_t snapshot_direction = last_pulse_direction;
  interrupts();

  bicycle_firmware::EncoderReading reading = {snapshot_counts, 0};
  if (snapshot_interval != 0) {
    // Report the time since the last pulse once it exceeds the last interval, so the speed
    // estimate decays while the wheel slows down or stops.
    const uint32_t since = micros() - snapshot_last;
    uint32_t period = snapshot_interval > since ? snapshot_interval : since;
    if (period > 2000000000UL) {
      period = 2000000000UL;
    }
    reading.period_us = snapshot_direction * static_cast<int32_t>(period);
  }
  return reading;
}

}  // namespace wheel_encoder
