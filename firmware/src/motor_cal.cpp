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

#include <Arduino.h>

#include "motor_cal.h"
#include "motor_control.h"

MotorCalData motor_cal = {0, {0, 0}, {0, 0}, {0, 0}};

// Three things are measured about this motor, per direction, and the MOVE_*
// gains in motor_control.h are re-derived from them (apply_motor_calibration):
//
//   breakaway  the duty at which the carriage first moves at all
//   k          ADC counts/sec of cruise gained per duty count above breakaway
//   v_jump     the speed motion starts at the instant breakaway is crossed
//
// See "Per-unit motor characterisation" in ABOUT_MOTOR_CONTROL.md for why a
// fixed set of gains can't serve every unit and how these are derived.

// Target open-loop gain for the velocity loop, k*KV, held constant across
// units by scaling KV with 1/k. 1.16 is what the reference unit ran at.
#define MOTORCAL_KV_LOOP_GAIN (1.16f)

// Bounds on the derived velocity floor (1.25x the Stribeck jump).
#define MOTORCAL_VEL_MIN_LO (200)
#define MOTORCAL_VEL_MIN_HI (1500)

// Time from the carriage reaching the deadband to actually stopping -
// reaction + detection + coast, not coast alone (see ABOUT_MOTOR_CONTROL.md).
#define MOTORCAL_STOP_TIME_MS (15)       // ms
#define MOTORCAL_TAKEUP_MARGIN (20)      // take-up ceiling sits this far above FF
// Bounds on the derived on-target window.
#define MOTORCAL_DEADBAND_LO (8)         // ADC counts
#define MOTORCAL_DEADBAND_HI (20)

// Measurement procedure: park, ramp to breakaway, then dwell at two duties
// above breakaway for a two-point fit of k. Runs alternate direction, so each
// one starts where the previous one left off.
#define MOTORCAL_PASSES (2)              // runs per direction, averaged
#define MOTORCAL_RAMP_RATE (80)          // duty/sec
// The ramp accumulates in fixed point at 1/256 duty per LSB, stepped per
// microsecond of tick as MOTORCAL_RAMP_STEP/4096. The 1/4096 is fine enough
// that a 500 us tick's increment doesn't truncate away most of itself.
#define MOTORCAL_RAMP_STEP \
  (((uint32_t)MOTORCAL_RAMP_RATE * 256 * 4096 + 500000UL) / 1000000UL)
#define MOTORCAL_RAMP_MAX (200)         // give up: this unit has no usable breakaway
#define MOTORCAL_MOTION_VEL (200)       // ADC counts/sec that counts as "moving"
#define MOTORCAL_JUMP_MS (40)            // window for capturing the Stribeck jump
#define MOTORCAL_DWELL_A (25)            // duty above breakaway, first fit point
#define MOTORCAL_DWELL_B (50)            // duty above breakaway, second fit point
#define MOTORCAL_DWELL_MS (100)          // hold per fit point
#define MOTORCAL_DWELL_AVG_MS (50)       // average over the last part of the hold
#define MOTORCAL_SETTLE_MS (200)         // coast to a stop before ramping
#define MOTORCAL_PARK_MARGIN (60)        // ADC counts from the end to start a run
#define MOTORCAL_PARK_DUTY (200)         // open-loop duty used to park
#define MOTORCAL_PARK_TIMEOUT (2500)     // ms
#define MOTORCAL_EDGE_MARGIN (25)        // abort if a run gets this close to an end

// Plausibility bounds - outside these, the measurement (not the fader) is
// wrong, so fall back to the compiled defaults.
#define MOTORCAL_BD_MIN (10)
#define MOTORCAL_BD_MAX (180)
#define MOTORCAL_K_MIN (8)
#define MOTORCAL_K_MAX (90)
#define MOTORCAL_VJUMP_MIN (100)
#define MOTORCAL_VJUMP_MAX (4000)

static uint32_t self_calibration_start = 0;
static uint8_t self_calibration_stage = 0;
#define SELF_CALIBRATION_TIMEOUT (1500)
#define SELF_CALIBRATION_BUFFER (0.995)  // Buffer factor to prevent hitting physical limits
static uint16_t self_calibration_adc_stage_0 = 0;
static uint16_t self_calibration_adc_stage_1 = 0;

// Self-calibration runs the endpoint sweep first, then characterises the motor
// (see the MOTORCAL_* constants). Stages 0-2 are the original endpoint sweep;
// their numbering is unchanged.
enum SelfCalStage : uint8_t {
  SELFCAL_ENDPOINT_LOW = 0,   // drive to the low end, record it
  SELFCAL_ENDPOINT_HIGH,      // drive to the high end, record it
  SELFCAL_ENDPOINT_APPLY,     // validate the span, adopt it in RAM
  SELFCAL_MOTOR_PARK,         // drive to the end this run starts from
  SELFCAL_MOTOR_SETTLE,       // coast to a genuine stop before measuring
  SELFCAL_MOTOR_RAMP,         // ramp to breakaway, then hold and capture the jump
  SELFCAL_MOTOR_DWELL_A,      // hold breakaway + A, measure cruise speed
  SELFCAL_MOTOR_DWELL_B,      // hold breakaway + B, measure cruise speed
  SELFCAL_MOTOR_NEXT,         // fold in the run, advance or finish
  SELFCAL_FINISH,             // derive gains, persist, return to the target
};

// Motor characterisation working state. Runs alternate direction starting with
// falling (the endpoint sweep leaves the carriage at the high end), so each run
// begins where the previous one ended.
static uint8_t motorcal_run = 0;          // 0..2*MOTORCAL_PASSES-1; even falling, odd rising
static bool motorcal_failed = false;
static uint16_t motorcal_duty = 0;        // ramped duty, fixed point (1/256 duty per LSB)
static int16_t motorcal_bd = 0;           // breakaway duty found by this run
static int16_t motorcal_vjump = 0;        // peak speed just after breakaway, ADC/s
static int16_t motorcal_dwell_a = 0;      // cruise speed at breakaway + A, ADC/s
static int32_t motorcal_vel_sum = 0;      // dwell velocity accumulator
static uint16_t motorcal_vel_n = 0;
// Per-direction accumulators, indexed [0] = falling, [1] = rising. Integer,
// because flash is the scarce resource on this part and a duty count or an
// ADC count/sec is all the resolution any of these needs.
static uint16_t motorcal_bd_sum[2] = {0, 0};
static uint16_t motorcal_k_sum[2] = {0, 0};
static uint16_t motorcal_vjump_max[2] = {0, 0};
static uint8_t motorcal_samples[2] = {0, 0};

// Re-derive the live control gains from this unit's measured motor
// characteristics, or restore the compiled-in defaults when no valid
// measurement exists. Safe to call at any time; it only touches gains. See
// "What is derived" in ABOUT_MOTOR_CONTROL.md for the formulas.
void apply_motor_calibration() {
  // The compiled-in defaults are themselves a plant model - the reference
  // unit's - so there is one derivation, not one per case: an uncharacterised
  // fader runs exactly the gains the constants were tuned to.
  uint8_t bd[2], k[2];
  uint16_t vjump;
  if (motor_cal.valid) {
    bd[MOTORCAL_FALLING] = motor_cal.bd[MOTORCAL_FALLING];
    bd[MOTORCAL_RISING] = motor_cal.bd[MOTORCAL_RISING];
    k[MOTORCAL_FALLING] = motor_cal.k[MOTORCAL_FALLING];
    k[MOTORCAL_RISING] = motor_cal.k[MOTORCAL_RISING];
    vjump = (motor_cal.vjump[0] > motor_cal.vjump[1]) ? motor_cal.vjump[0]
                                                      : motor_cal.vjump[1];
  } else {
    bd[MOTORCAL_FALLING] = MOVE_BD_FALLING;
    bd[MOTORCAL_RISING] = MOVE_BD_RISING;
    k[MOTORCAL_FALLING] = MOVE_K_DEFAULT;
    k[MOTORCAL_RISING] = MOVE_K_DEFAULT;
    vjump = MOVE_VJUMP_DEFAULT;
  }

  // The Stribeck jump is the slowest the mechanism moves at all, so it floors
  // both the velocity reference and the on-target window: the fader cannot
  // correct by less than the distance it covers before it can be stopped.
  uint16_t vel_min = vjump + vjump / 4;  // 1.25x, for margin above the floor
  if (vel_min < MOTORCAL_VEL_MIN_LO) vel_min = MOTORCAL_VEL_MIN_LO;
  if (vel_min > MOTORCAL_VEL_MIN_HI) vel_min = MOTORCAL_VEL_MIN_HI;
  move_vel_min = vel_min;

  uint16_t deadband = (uint16_t)(((uint32_t)vel_min * MOTORCAL_STOP_TIME_MS + 500) / 1000);
  if (deadband < MOTORCAL_DEADBAND_LO) deadband = MOTORCAL_DEADBAND_LO;
  if (deadband > MOTORCAL_DEADBAND_HI) deadband = MOTORCAL_DEADBAND_HI;
  move_deadband = deadband;

  // Take-up ceiling: limits drive at the start of a move so the belt is not
  // engaged at full duty. It has to stay above the feedforward at the floor
  // speed, or it starves the term that gets the carriage moving. Integer
  // arithmetic deliberately - this runs once, but flash is permanent.
  int16_t takeup = MOVE_TAKEUP_DUTY;
  for (uint8_t i = 0; i < 2; i++) {
    int16_t ff = bd[i] + (int16_t)(vel_min / (k[i] ? k[i] : 1)) + MOTORCAL_TAKEUP_MARGIN;
    if (ff > takeup) takeup = ff;
  }
  if (takeup > MOVE_MAX_DUTY) takeup = MOVE_MAX_DUTY;
  move_takeup_duty = takeup;

  // The only float work: the control law needs 1/k per direction for the
  // feedforward, and the velocity loop gain scaled so its open-loop gain k*KV
  // - the thing that actually sets damping - matches the reference unit's.
  uint16_t k_avg = ((uint16_t)k[0] + k[1] + 1) / 2;
  if (k_avg < MOTORCAL_K_MIN) k_avg = MOTORCAL_K_MIN;
  if (k_avg > MOTORCAL_K_MAX) k_avg = MOTORCAL_K_MAX;
  move_kv = MOTORCAL_KV_LOOP_GAIN / (float)k_avg;
  for (uint8_t i = 0; i < 2; i++) {
    move_bd[i] = bd[i];
    move_inv_k[i] = 1.0f / (float)(k[i] ? k[i] : 1);
  }
}

// Direction of travel for the current characterisation run: runs alternate,
// starting with falling, so each begins where the previous one ended.
static int8_t motorcal_dir() {
  return (motorcal_run & 1) ? 1 : -1;
}

// A run drives open-loop, so it has to police its own travel. Bail out before
// the carriage reaches the mechanical end rather than measuring a fader that
// is being held by an endstop.
static bool motorcal_out_of_band(int8_t dir) {
  if (dir > 0) return input_ewma > input_calib_max - MOTORCAL_EDGE_MARGIN;
  return input_ewma < input_calib_min + MOTORCAL_EDGE_MARGIN;
}

// Start a self-calibration run at the endpoint sweep.
void motorcal_begin(uint32_t now) {
  self_calibration_stage = SELFCAL_ENDPOINT_LOW;
  self_calibration_start = now;
}

MotorCalResult motorcal_tick(uint32_t now, uint16_t adc_raw) {
  // Direction of travel and travel policing are the same for every
  // measurement stage, so they live here rather than in each case. The
  // runs drive open loop, so something has to stop them at the ends.
  int8_t dir = motorcal_dir();
  // Speed along the direction of travel, as an integer once, so the
  // measurement stages below never touch the float paths again. One ADC
  // count/sec is far finer than anything derived from these needs.
  int16_t speed = (int16_t)((dir > 0) ? velocity_ewma : -velocity_ewma);
  if (self_calibration_stage >= SELFCAL_MOTOR_RAMP &&
      self_calibration_stage <= SELFCAL_MOTOR_DWELL_B &&
      motorcal_out_of_band(dir)) {
    motorcal_failed = true;
    self_calibration_stage = SELFCAL_MOTOR_NEXT;
  }
  switch (self_calibration_stage) {
    case SELFCAL_ENDPOINT_LOW:
      if (now > self_calibration_start + SELF_CALIBRATION_TIMEOUT) {
        self_calibration_adc_stage_0 = adc_raw;
        self_calibration_stage++;
        self_calibration_start = millis();
      } else {
        // Move toward lower ADC value
        motor_set(-254, true, MOTOR_IDLE_COAST);
      }
      break;
    case SELFCAL_ENDPOINT_HIGH:
      if (now > self_calibration_start + SELF_CALIBRATION_TIMEOUT) {
        self_calibration_adc_stage_1 = adc_raw;
        self_calibration_stage++;
        self_calibration_start = millis();
      } else {
        // Move toward higher ADC value
        motor_set(254, true, MOTOR_IDLE_COAST);
      }
      break;
    case SELFCAL_ENDPOINT_APPLY:
      motor_coast();
      if (abs((int16_t)self_calibration_adc_stage_0 - self_calibration_adc_stage_1) < 900) {
        return MOTORCAL_BAD_SPAN;
      } else {
        // Adopt the measured span, pulled in by the buffer factor so the
        // ends stay clear of the mechanical stops. Halved because the
        // stage readings are raw (2x accumulation) and the bounds are
        // compared against input_ewma, which is in ADC counts.
        input_calib_min = (SELF_CALIBRATION_BUFFER * self_calibration_adc_stage_0 + (1.0 - SELF_CALIBRATION_BUFFER) * self_calibration_adc_stage_1) / 2;
        input_calib_max = (SELF_CALIBRATION_BUFFER * self_calibration_adc_stage_1 + (1.0 - SELF_CALIBRATION_BUFFER) * self_calibration_adc_stage_0) / 2;

        // Endpoints are known, so the motor characterisation that follows
        // has a calibrated span to keep itself inside. Persist once, at the
        // end, so endpoints and motor data are written together.
        motorcal_run = 0;
        motorcal_failed = false;
        for (uint8_t i = 0; i < 2; i++) {
          motorcal_bd_sum[i] = 0;
          motorcal_k_sum[i] = 0;
          motorcal_vjump_max[i] = 0;
          motorcal_samples[i] = 0;
        }
        self_calibration_stage = SELFCAL_MOTOR_PARK;
        self_calibration_start = now;
      }
      break;
    case SELFCAL_MOTOR_PARK: {
      // A run needs room to ramp to breakaway and then dwell twice without
      // reaching the far end, so it starts from the end it travels away
      // from. Runs alternate direction, so only the first park is long.
      bool parked = (dir > 0)
          ? (input_ewma <= input_calib_min + MOTORCAL_PARK_MARGIN)
          : (input_ewma >= input_calib_max - MOTORCAL_PARK_MARGIN);
      if (parked || now > self_calibration_start + MOTORCAL_PARK_TIMEOUT) {
        motor_brake();
        self_calibration_stage = SELFCAL_MOTOR_SETTLE;
        self_calibration_start = now;
      } else {
        motor_set(dir > 0 ? -MOTORCAL_PARK_DUTY : MOTORCAL_PARK_DUTY,
                  true, MOTOR_IDLE_COAST);
      }
      break;
    }
    case SELFCAL_MOTOR_SETTLE:
      // Breakaway is only meaningful from a standstill: measuring it while
      // the carriage still coasts would find the (much lower) duty needed
      // to sustain motion rather than the duty needed to start it.
      motor_brake();
      if (now > self_calibration_start + MOTORCAL_SETTLE_MS) {
        motorcal_duty = 0;
        motorcal_bd = 0;
        motorcal_vjump = 0;
        self_calibration_stage = SELFCAL_MOTOR_RAMP;
        self_calibration_start = now;
      }
      break;
    case SELFCAL_MOTOR_RAMP: {
      int16_t duty;
      if (motorcal_bd == 0) {
        // Still hunting for breakaway. Fixed point (see MOTORCAL_RAMP_STEP):
        // the ramp is the only thing here needing sub-duty resolution, and a
        // float multiply per tick for it is not worth the flash. Truncation
        // per tick costs a little rate - 78 duty/sec realised at the 500 us
        // tick against the nominal 80 - which the breakaway search absorbs.
        motorcal_duty += (uint16_t)((control_dt_us * MOTORCAL_RAMP_STEP) >> 12);
        duty = motorcal_duty >> 8;
        if (duty > MOTORCAL_RAMP_MAX) {
          // Nothing moved short of a duty no sane fader needs: jammed, or
          // the motor isn't connected.
          motorcal_failed = true;
          self_calibration_stage = SELFCAL_MOTOR_NEXT;
          break;
        }
        // duty > 0 keeps a still-coasting carriage from "breaking away" at
        // zero drive, which would also defeat the sentinel below.
        if (duty > 0 && speed > MOTORCAL_MOTION_VEL) {
          motorcal_bd = duty;
          motorcal_vjump = speed;
          self_calibration_start = now;  // start the jump hold
        }
      } else {
        // Breakaway found (motorcal_bd is the sentinel; it is never 0 for
        // a real detection). Hold there and take the peak speed. Motion does
        // not ease in from zero - it jumps (Stribeck) - and that jump is
        // the floor on the smallest correction the controller can make.
        duty = motorcal_bd;
        if (speed > motorcal_vjump) motorcal_vjump = speed;
        if (now > self_calibration_start + MOTORCAL_JUMP_MS) {
          motorcal_vel_sum = 0;
          motorcal_vel_n = 0;
          self_calibration_stage = SELFCAL_MOTOR_DWELL_A;
          self_calibration_start = now;
        }
      }
      motor_set((dir > 0) ? duty : -duty, true, MOTOR_IDLE_BRAKE);
      break;
    }
    case SELFCAL_MOTOR_DWELL_A:
    case SELFCAL_MOTOR_DWELL_B: {
      // Two fixed duties above breakaway give a two-point fit for k, the
      // slope of the speed/duty line. One point would only give a speed,
      // which says nothing about how the plant responds to a change.
      bool second = (self_calibration_stage == SELFCAL_MOTOR_DWELL_B);
      int16_t duty = motorcal_bd + (second ? MOTORCAL_DWELL_B : MOTORCAL_DWELL_A);
      if (duty > MOVE_MAX_DUTY) duty = MOVE_MAX_DUTY;
      motor_set((dir > 0) ? duty : -duty, true, MOTOR_IDLE_BRAKE);

      // Average the tail of the hold only, once the speed has settled.
      if (now > self_calibration_start + (MOTORCAL_DWELL_MS - MOTORCAL_DWELL_AVG_MS)) {
        motorcal_vel_sum += speed;
        motorcal_vel_n++;
      }
      if (now > self_calibration_start + MOTORCAL_DWELL_MS) {
        int16_t v = (motorcal_vel_n > 0)
                        ? (int16_t)(motorcal_vel_sum / motorcal_vel_n)
                        : 0;
        motorcal_vel_sum = 0;
        motorcal_vel_n = 0;
        if (!second) {
          motorcal_dwell_a = v;
          self_calibration_stage = SELFCAL_MOTOR_DWELL_B;
          self_calibration_start = now;
        } else {
          int16_t k = (v - motorcal_dwell_a) / (MOTORCAL_DWELL_B - MOTORCAL_DWELL_A);
          int16_t bd = motorcal_bd;
          int16_t vjump = motorcal_vjump;
          uint8_t idx = (dir > 0) ? MOTORCAL_RISING : MOTORCAL_FALLING;
          if (bd >= MOTORCAL_BD_MIN && bd <= MOTORCAL_BD_MAX &&
              k >= MOTORCAL_K_MIN && k <= MOTORCAL_K_MAX &&
              vjump >= MOTORCAL_VJUMP_MIN && vjump <= MOTORCAL_VJUMP_MAX) {
            motorcal_bd_sum[idx] += bd;
            motorcal_k_sum[idx] += k;
            if ((uint16_t)vjump > motorcal_vjump_max[idx]) {
              motorcal_vjump_max[idx] = vjump;
            }
            motorcal_samples[idx]++;
          } else {
            motorcal_failed = true;
          }
          self_calibration_stage = SELFCAL_MOTOR_NEXT;
          self_calibration_start = now;
        }
      }
      break;
    }
    case SELFCAL_MOTOR_NEXT:
      motor_brake();
      motorcal_run++;
      self_calibration_stage =
          (!motorcal_failed && motorcal_run < 2 * MOTORCAL_PASSES)
              ? SELFCAL_MOTOR_PARK
              : SELFCAL_FINISH;
      self_calibration_start = now;
      break;
    case SELFCAL_FINISH:
      motor_coast();
      // Every run must have produced a plausible sample. A partial set
      // means something was wrong with the fader or the measurement, and
      // half-measured gains are worse than the tuned defaults.
      if (!motorcal_failed && motorcal_samples[MOTORCAL_FALLING] == MOTORCAL_PASSES &&
          motorcal_samples[MOTORCAL_RISING] == MOTORCAL_PASSES) {
        motor_cal.valid = 1;
        for (uint8_t i = 0; i < 2; i++) {
          motor_cal.bd[i] = (motorcal_bd_sum[i] + MOTORCAL_PASSES / 2) / MOTORCAL_PASSES;
          motor_cal.k[i] = (motorcal_k_sum[i] + MOTORCAL_PASSES / 2) / MOTORCAL_PASSES;
          motor_cal.vjump[i] = motorcal_vjump_max[i];
        }
      } else {
        // Endpoints are still good and worth keeping; only the motor
        // characterisation is discarded, and the tuned defaults stand in.
        motor_cal.valid = 0;
        for (uint8_t i = 0; i < 2; i++) {
          motor_cal.bd[i] = 0;
          motor_cal.k[i] = 0;
          motor_cal.vjump[i] = 0;
        }
      }
      apply_motor_calibration();

      // The caller persists the record and returns the carriage to its
      // target; both need state this module has no business reaching into.
      return MOTORCAL_DONE;
  }
  return MOTORCAL_RUNNING;
}
