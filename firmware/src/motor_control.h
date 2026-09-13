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

// Motor drive primitives and the remote-movement control law's tuning, shared
// between the control loop in main.cpp (which owns every definition here) and
// the per-unit characterisation in motor_cal.cpp (which derives the live gains
// from a measurement of the plant).

#pragma once

#include <stdint.h>

// PA4 = Motor A (WO4/HCMP1), PA5 = Motor B (WO5/HCMP2). Both feed the DRV8837
// IN1/IN2 inputs: 1/0 = forward, 0/1 = reverse, 0/0 = coast (Hi-Z), 1/1 = brake.
enum MotorIdle : uint8_t {
  MOTOR_IDLE_COAST = 0,  // Bridge Hi-Z: fader is free to move
  MOTOR_IDLE_BRAKE = 1,  // Both low sides on: dynamic braking against motion
};

void motor_set(int16_t drive, bool slow_decay, MotorIdle idle_mode);
void motor_coast();
void motor_brake();

// Direction of travel, used to index every per-direction quantity here and in
// the characterisation. "Rising" is toward the motor end, where ADC increases.
#define MOTORCAL_FALLING (0)
#define MOTORCAL_RISING (1)

// ---------------------------------------------------------------------------
// Remote-movement control law, in ADC counts. Grouped by stage of the cascade,
// matching "The control law" in ABOUT_MOTOR_CONTROL.md - read the two together
// for the derivation and for how these were centred for hardware variance.
// ---------------------------------------------------------------------------

// Position loop: error -> velocity reference.
#define MOVE_VREF_SLOPE (50.0f)         // ADC counts/sec of reference per ADC count of error
// The mechanism stick-slips below the Stribeck floor rather than moving, so the
// reference is never allowed under it. Holding it there next to the target is
// also what keeps the feedforward clear of breakaway for small errors.
#define MOVE_VEL_MIN (560)              // ADC counts/sec
#define MOVE_VEL_UNLIMITED (0.0f)       // move_max_velocity: no speed limit requested

// Friction feedforward: u_ff = breakaway + v_ref/k, the plant inverted. These
// are the reference unit's measured plant; a characterised unit overrides them.
#define MOVE_BD_RISING (68)             // assumed breakaway duty, rising ADC
#define MOVE_BD_FALLING (86)            // assumed breakaway duty, falling ADC
#define MOVE_K_DEFAULT (29)             // assumed ADC counts/sec per duty count
#define MOVE_VJUMP_DEFAULT (448)        // assumed Stribeck jump; 1.25x it is MOVE_VEL_MIN

// Velocity loop: closes on the reference. What sets damping is the open-loop
// gain k*KV, so this is scaled by 1/k when the unit has been characterised.
#define MOVE_KV (0.04f)                 // duty per (ADC count/sec) of velocity error

// Stall escape: extra drive ramped in only while commanded to move but not
// moving, so a unit whose breakaway sits above the feedforward still completes
// a move instead of sitting stuck outside the deadband until the timeout.
#define MOVE_STALL_VELOCITY (60.0f)     // ADC counts/sec below which we're stalled
#define MOVE_RAMP_RATE (250.0f)         // duty per second of ramp-in
#define MOVE_RAMP_MAX (70.0f)           // ceiling, so a jam can't wind up to full drive
#define MOVE_RAMP_DECAY_RATE (3500.0f)  // duty/sec shed once moving (~20 ms from full)

// Backlash take-up: a direction-reversed move starts with the motor unloaded,
// so full duty would spin the rotor through the belt slack and snap it taut -
// an audible click and a jerk. The drive ceiling instead starts low and opens
// up over time. Time-based, not "are we moving yet": crossing the slack the
// rotor is already moving, just unloaded. Must stay above the feedforward or it
// starves the term that gets the carriage moving.
#define MOVE_TAKEUP_DUTY (130)           // duty ceiling when a move is armed
#define MOVE_TAKEUP_RAMP_RATE (1200.0f)  // duty per second the ceiling opens up

// Move termination. The deadband has a hard floor set by the plant, not by
// taste - see ABOUT_MOTOR_CONTROL.md. Don't lower it below that floor.
#define MOVE_DEADBAND (8.0f)            // ADC counts considered "on target"
#define MOVE_MAX_DUTY (254)
// On movement timeout, an error below this means the fader arrived but is
// still dithering - go idle instead of latching MODE_ERROR.
#define MOVE_TIMEOUT_TOLERANCE (20.0f)  // ADC counts

// Endpoints of the host-facing unitless speed scale (LAYER_SPEED_SLOWEST ..
// LAYER_SPEED_FULL - 1), as the full-travel time at each end - the validated
// range of the speed limiter. See "Optional speed limiting" in
// ABOUT_MOTOR_CONTROL.md.
#define MOVE_SPEED_SLOWEST_MS (700)
#define MOVE_SPEED_FASTEST_MS (250)

// Live gains. The MOVE_* constants above are the defaults for a unit that has
// never been characterised; apply_motor_calibration() (motor_cal.cpp) derives
// these from a real measurement when one exists, and DEBUG_DRIVE builds let a
// host overwrite them at runtime via REG_DEBUG_GAINS.
extern float move_bd[2];         // breakaway duty, per direction
extern float move_inv_k[2];      // duty per (ADC count/sec), per direction
extern float move_kv;            // velocity loop gain
extern int16_t move_takeup_duty;
extern float move_deadband;
extern float move_vel_min;

// Measured interval of the current control tick, in microseconds. Measured from
// the previous execution rather than the scheduled tick time, so a late tick
// doesn't skew anything derived from it.
extern uint32_t control_dt_us;

// Plant state the characterisation measures against.
extern float input_ewma;         // filtered position, ADC counts
extern float velocity_ewma;      // ADC counts/sec, + toward the motor end
extern uint16_t input_calib_min;
extern uint16_t input_calib_max;
