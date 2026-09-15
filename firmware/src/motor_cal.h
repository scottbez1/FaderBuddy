/*
 * Copyright 2026 Scott Bezek
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Per-unit motor characterisation: measures this fader's plant during
// self-calibration and re-derives the control law's gains from it. See
// "Per-unit motor characterisation" in ABOUT_MOTOR_CONTROL.md for why a single
// fixed set of gains can't serve every unit, and how these are derived.

#pragma once

#include <stdint.h>

// What was measured, as reported at REG_MOTOR_CAL. Held separately from the
// derived gains so the raw measurement stays inspectable, and persisted in the
// same EEPROM record as the endpoints. Indexed [MOTORCAL_FALLING]/[MOTORCAL_RISING].
struct MotorCalData {
  uint8_t valid;      // 0 = never measured, or measured implausibly
  uint8_t bd[2];      // breakaway duty
  uint8_t k[2];       // ADC counts/sec per duty above breakaway
  uint16_t vjump[2];  // ADC counts/sec at the moment motion starts
};

extern MotorCalData motor_cal;

// Re-derive the live control gains from motor_cal, or restore the compiled-in
// defaults when it holds no valid measurement. Safe to call at any time; it
// only touches gains.
void apply_motor_calibration();

enum MotorCalResult : uint8_t {
  MOTORCAL_RUNNING = 0,  // still sweeping or measuring
  MOTORCAL_DONE,         // motor_cal and the endpoints are settled; persist them
  MOTORCAL_BAD_SPAN,     // the endpoint sweep found no plausible travel
};

// Start a self-calibration run: endpoint sweep first, then the motor
// characterisation. The caller owns the mode; this owns the motor until it
// returns something other than MOTORCAL_RUNNING.
void motorcal_begin(uint32_t now);

// Advance one control tick. `adc_raw` is the latest raw (2x accumulated)
// conversion, used for the endpoint sweep.
MotorCalResult motorcal_tick(uint32_t now, uint16_t adc_raw);
