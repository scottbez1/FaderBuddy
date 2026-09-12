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
#include <util/delay.h>
#include <Wire.h>
#include <ptc_touch.h>
#include <EEPROM.h>

#include "shared/i2c_data.h"
#include "util.h"



#define DEMO 0

#define PIN_LED (PIN_PB2)
#define LED_bm (1 << 2)  // PIN_LED on VPORTB, for direct port access

#define PIN_MOTOR_nSLEEP (PIN_PB3)
#define MOTOR_nSLEEP_bm (1 << 3)  // PIN_MOTOR_nSLEEP on VPORTB, for direct port access

// Energizing pin A moves fader toward the motor end
#define PIN_MOTOR_A (PIN_PA4)
#define PIN_MOTOR_B (PIN_PA5)

// Value increases as fader approaches the motor end
#define PIN_FADER (PIN_PA6)

#define PIN_TOUCH (PIN_PC3)

#define PIN_ADDR_0 (PIN_PC2)
#define PIN_ADDR_1 (PIN_PC1)
#define PIN_ADDR_2 (PIN_PC0)


// PWM configuration
#if defined(MILLIS_USE_TIMERA0) || defined(__AVR_ATtinyxy2__)
  #error "This sketch takes over TCA0, don't use for millis here.  Pin mappings on 8-pin parts are different"
#endif

#define MOVEMENT_TIMEOUT_MILLIS (8000)
#define TOUCH_OVERRIDE_DURATION_THRESHOLD (50)
#define REMOTE_MOVEMENT_STEADY_THRESHOLD (300)
#define IDLE_DURATION_THRESHOLD (1000)

// Haptic parameters
#define HAPTIC_DEAD_ZONE (8)           // ADC units of dead zone around target
#define HAPTIC_BASE_PWM (150)           // Base PWM value for haptic force
#define HAPTIC_MAX_PWM (254)            // Maximum PWM value
#define HAPTIC_MAGNET_RANGE (60)        // Active range for magnetic endpoints (ADC units)
#define HAPTIC_BASE_MULTIPLIER (3.0f)   // Base force multiplier for haptics

// Tap detection timing
#define TAP_MAX_DURATION (200)            // Maximum tap press duration (ms)
#define DOUBLE_TAP_MAX_INTERVAL (200)     // Maximum time between taps (ms)
#define TAP_MAX_MOVEMENT (10)             // Maximum position change during tap (ADC units)

// Tap detection state machine
enum TapState : uint8_t {
  TAP_NONE = 0,                // No tap in progress
  TAP_FIRST_PRESSED,           // First tap touch detected
  TAP_WAITING_FOR_DOUBLE,      // Waiting to see if second tap occurs
  TAP_SECOND_PRESSED,          // Second tap touch detected
};

// Fixed-rate control tick, so gains and filter time constants keep the same
// meaning regardless of how long touch processing takes on a given loop().
#define CONTROL_TICK_US (500)           // 2 kHz (leaves loop headroom for touch)
#define VELOCITY_TAU_S (0.004f)         // velocity estimate smoothing, seconds

// Remote-movement control gains, in ADC counts. Cascade control: the position
// loop turns error into a velocity reference (MOVE_VREF_SLOPE), the velocity
// loop (MOVE_KV) realises it, and MOVE_BD_*/MOVE_K_DEFAULT feed forward the
// plant's friction. See "The control law" in ABOUT_MOTOR_CONTROL.md for the
// derivation and how these were tuned for hardware variance.
#define MOVE_VREF_SLOPE (50.0f)         // ADC counts/sec of reference per ADC count of error
#define MOVE_KV (0.04f)                 // duty per (ADC count/sec) of velocity error; default only, scaled by 1/k when characterised
// Friction feedforward: u_ff = breakaway + v_ref/k. Defaults are the
// reference unit's measured plant; a characterised unit overrides them.
#define MOVE_BD_RISING (68)             // assumed breakaway duty, rising ADC
#define MOVE_BD_FALLING (86)            // assumed breakaway duty, falling ADC
#define MOVE_K_DEFAULT (29)             // assumed ADC counts/sec per duty count
#define MOVE_VJUMP_DEFAULT (448)        // assumed Stribeck jump; 1.25x it is MOVE_VEL_MIN
// On-target window. Has a hard floor set by the plant, not by taste - see
// ABOUT_MOTOR_CONTROL.md. Don't lower it below the floor.
#define MOVE_DEADBAND (8.0f)            // ADC counts considered "on target"
#define MOVE_MAX_DUTY (254)

// MOVE_VEL_MIN floors the velocity reference: below it the mechanism
// stick-slips instead of moving smoothly (the Stribeck floor), and it's what
// keeps the feedforward clear of breakaway when the error is small.
#define MOVE_VEL_MIN (560)            // ADC counts/sec
#define MOVE_VEL_UNLIMITED (0.0f)     // move_max_velocity: no speed limit requested

// Endpoints of the host-facing unitless speed scale (LAYER_SPEED_SLOWEST ..
// LAYER_SPEED_FULL - 1), as the full-travel time at each end - the validated
// range of the speed limiter. See "Optional speed limiting" in
// ABOUT_MOTOR_CONTROL.md.
#define MOVE_SPEED_SLOWEST_MS (700)
#define MOVE_SPEED_FASTEST_MS (250)

// On movement timeout, an error below this means the fader arrived but is
// still dithering - go idle instead of latching MODE_ERROR.
#define MOVE_TIMEOUT_TOLERANCE (20.0f)  // ADC counts

// Stiction ramp: extra drive added while commanded to move but stalled, so a
// unit whose breakaway sits above the feedforward can still complete a move.
#define MOVE_STALL_VELOCITY (60.0f)     // ADC counts/sec below which we're stalled
#define MOVE_RAMP_RATE (250.0f)         // duty per second of ramp-in
#define MOVE_RAMP_MAX (70.0f)           // ceiling, so a jam can't wind up to full drive
#define MOVE_RAMP_DECAY_RATE (3500.0f)  // duty/sec shed once moving (~20 ms from full)

// Backlash take-up: a direction-reversed move starts with the motor
// unloaded, so full duty would spin the rotor through the belt slack and
// snap it taut - an audible click and a jerk. The drive ceiling instead
// starts low at the beginning of every move and opens up over time (this has
// to be time-based, not "are we moving yet", since crossing the slack the
// rotor is already moving but unloaded). Must stay above the feedforward
// (MOVE_BD_RISING/FALLING plus headroom) or it starves the term that gets
// the carriage moving.
#define MOVE_TAKEUP_DUTY (130)           // duty ceiling at the start of a move
#define MOVE_TAKEUP_RAMP_RATE (1200.0f)  // duty per second the ceiling opens up

// ---------------------------------------------------------------------------
// Per-unit motor characterisation
// ---------------------------------------------------------------------------
// Self-calibration measures three things about this motor, per direction, and
// the gains above are re-derived from them (apply_motor_calibration):
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
#define MOTORCAL_RAMP_RATE (80.0f)       // duty/sec
#define MOTORCAL_RAMP_MAX (200.0f)       // give up: this unit has no usable breakaway
#define MOTORCAL_MOTION_VEL (200.0f)     // ADC counts/sec that counts as "moving"
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

// The MOVE_* constants above are defaults for a unit that has never been
// motor-calibrated; apply_motor_calibration() derives the live values below
// from a real measurement when one exists.
//
// DEBUG_DRIVE builds additionally let a host overwrite these at runtime via
// REG_DEBUG_GAINS, so a gain sweep doesn't need a reflash per trial.
// Indexed [MOTORCAL_FALLING]/[MOTORCAL_RISING] where per-direction.
float move_bd[2];                                          // breakaway duty
float move_inv_k[2];                                       // duty per (ADC/s)
float move_kv = MOVE_KV;                                   // velocity loop gain
int16_t move_takeup_duty = MOVE_TAKEUP_DUTY;
float move_deadband = MOVE_DEADBAND;
float move_vel_min = MOVE_VEL_MIN;
#if DEBUG_DRIVE
// Runtime-tunable copies, so gain sweeps don't need a reflash per trial.
float move_vref_slope = MOVE_VREF_SLOPE;
float move_ramp_rate = MOVE_RAMP_RATE;
float move_takeup_ramp_rate = MOVE_TAKEUP_RAMP_RATE;
#else
#define move_vref_slope MOVE_VREF_SLOPE
#define move_ramp_rate MOVE_RAMP_RATE
#define move_takeup_ramp_rate MOVE_TAKEUP_RAMP_RATE
#endif

const float ALPHA = 0.05;
float input_ewma = 0;
float stiction_ramp = 0;                // extra drive ramped in while stalled
float move_max_velocity = MOVE_VEL_UNLIMITED;  // 0 = unlimited
float drive_ceiling = MOVE_MAX_DUTY;    // take-up ceiling, reset at each move start
float velocity_ewma = 0;                // ADC counts/sec, + toward the motor end
float last_control_ewma = 0;
uint32_t last_control_tick_us = 0;
uint32_t last_control_exec_us = 0;
uint32_t control_dt_us = CONTROL_TICK_US;  // measured interval of the current tick
#if DEBUG_DRIVE
uint16_t control_tick_period_us = CONTROL_TICK_US;  // runtime-tunable for sweeps
#else
#define control_tick_period_us CONTROL_TICK_US
#endif

// Touch state
bool touch = false;
cap_sensor_t touch_sensor;
uint16_t touch_recal_count = 0;  // Count of touch recalibrations since boot

// I2C slave base address (before A0/A1/A2 jumpers are applied)
const uint8_t I2C_BASE_ADDRESS = 0x20;

// EEPROM calibration storage. The magic changes whenever the layout does; an
// older record simply fails validation and the unit falls back to defaults
// until self-calibration is run again, which it needs anyway.
#define EEPROM_CALIBRATION_ADDR 0
#define EEPROM_CALIBRATION_MAGIC 0xCAF1  // Magic number to validate EEPROM data

// Motor characterisation, as measured and reported at REG_MOTOR_CAL. Held
// separately from the derived gains so the raw measurement stays inspectable.
// Indexed by direction of travel throughout: [MOTORCAL_FALLING]/[MOTORCAL_RISING].
#define MOTORCAL_FALLING (0)
#define MOTORCAL_RISING (1)

struct MotorCalData {
  uint8_t valid;      // 0 = never measured, or measured implausibly
  uint8_t bd[2];      // breakaway duty
  uint8_t k[2];       // ADC counts/sec per duty above breakaway
  uint16_t vjump[2];  // ADC counts/sec at the moment motion starts
};

struct CalibrationData {
  uint16_t magic;         // Magic number for validation
  uint16_t calib_min;     // Minimum ADC value (fader at one end)
  uint16_t calib_max;     // Maximum ADC value (fader at other end)
  MotorCalData motor;     // Per-unit motor characterisation (valid=0 if absent)
  uint16_t checksum;      // Simple checksum for data integrity
};

uint16_t input_calib_min = 40;
uint16_t input_calib_max = 1010;
MotorCalData motor_cal = {0, {0, 0}, {0, 0}, {0, 0}};

int16_t target_adc = 512;
uint8_t current_register = REG_VERSION;  // Track which register was last accessed

// Haptic configuration storage
uint32_t haptic_config = 0;  // Bit-packed haptic configuration register (currently active layer's config)
uint8_t last_haptic_nonce = 0;  // Track last seen nonce to detect changes [DEPRECATED in v5]

// Layer state storage (27 bytes total) - Protocol v5+
uint16_t layer_haptic_configs[8];      // 16 bytes - 16-bit haptic config per layer
uint8_t layer_restore_positions[8];    // 8 bytes - restore position per layer (0-255)
uint8_t layer_speeds[8];               // 8 bytes - per-layer move speed (see LAYER_SPEED_*)
uint8_t active_layer = 0;              // 1 byte - currently active layer (0-7)
uint8_t pending_layer_change = 0xFF;   // 1 byte - deferred layer change (0xFF = none, 0-7 = layer)
uint8_t queried_layer = 0;             // 1 byte - for layer-addressed read protocol

const int16_t WINDOW_SIZE = 8;
int16_t position_window_upper = WINDOW_SIZE;
int16_t position_window_lower = 0;
int16_t position = 0;

uint32_t state = (Mode::MODE_INPUT_IDLE << STATE_MODE_bp);

// ============================================================================
// ISR <-> Main Loop Communication
// ============================================================================
// All variables used to pass data between ISR and main loop are prefixed with i2c_

// Main loop -> ISR communication (outgoing state for I2C reads)
volatile uint32_t i2c_outgoing_state = state;

// ISR -> Main loop communication (incoming requests from I2C writes)
volatile bool i2c_clear_error_request = false;
volatile bool i2c_self_cal_request = false;
volatile uint8_t i2c_layer_change_request = 0xFF;  // 0xFF = none, 0-7 = layer

struct LayerTargetWrite {
  uint8_t layer;
  uint8_t target;
  uint8_t speed;     // optional move speed (see LAYER_SPEED_*)
  bool has_speed;    // false = 3-byte write, leave the layer's speed as-is
  bool valid;
};
volatile LayerTargetWrite i2c_layer_target_write = {0, 0, LAYER_SPEED_FULL, false, false};

struct LayerHapticWrite {
  uint8_t layer;
  uint16_t config;
  bool valid;
};
volatile LayerHapticWrite i2c_layer_haptic_write = {0, 0, false};

#if DEBUG_DRIVE
// ---------------------------------------------------------------------------
// Open-loop drive register (system identification only; not built into the
// production firmware). Lets a host command a raw duty/decay/PWM-frequency
// combination so duty->velocity and friction breakaway can be measured
// directly instead of inferred from closed-loop behaviour.
// ---------------------------------------------------------------------------
#define DEBUG_DRIVE_WATCHDOG_MS (400)  // Auto-coast if the host stops refreshing
#define DEBUG_DRIVE_EDGE_MARGIN (30)   // ADC counts of endstop keep-out

volatile bool i2c_debug_gain_valid = false;
volatile uint8_t i2c_debug_gain_index = 0;
volatile int16_t i2c_debug_gain_value = 0;

volatile bool i2c_debug_drive_valid = false;
volatile uint8_t i2c_debug_drive_flags = 0;
volatile uint8_t i2c_debug_drive_duty = 0;

bool debug_drive_active = false;
int16_t debug_drive_value = 0;
bool debug_drive_slow_decay = false;
bool debug_drive_brake = false;
uint32_t debug_drive_last_update = 0;

// Snapshot of control-loop internals for REG_DEBUG_STATUS
float debug_status_velocity = 0;
float debug_status_error = 0;
uint16_t debug_status_loop_hz = 0;
uint16_t debug_status_tick_hz = 0;
uint16_t debug_loop_count = 0;
uint16_t debug_tick_count = 0;
uint32_t debug_rate_window_start = 0;
#endif

uint32_t remote_movement_start = 0;
uint32_t touch_state_change_millis = 0;
uint32_t remote_movement_steady_start = 0;
uint32_t input_last_change_millis = 0;
uint16_t remote_movement_start_position = 0;

// Tap detection state
TapState tap_state = TAP_NONE;
uint32_t tap_timestamp = 0;          // Timestamp of last tap-related event for duration/timeout tracking
uint16_t tap_position_start = 0;     // Raw ADC value when first touch started
uint8_t double_tap_nonce = 0;

uint32_t self_calibration_start = 0;
uint8_t self_calibration_stage = 0;
#define SELF_CALIBRATION_TIMEOUT (1500)
#define SELF_CALIBRATION_BUFFER (0.995)  // Buffer factor to prevent hitting physical limits
uint16_t self_calibration_adc_stage_0 = 0;
uint16_t self_calibration_adc_stage_1 = 0;

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
uint8_t motorcal_run = 0;          // 0..2*MOTORCAL_PASSES-1; even falling, odd rising
bool motorcal_failed = false;
uint16_t motorcal_duty = 0;        // ramped duty, fixed point (1/64 duty per LSB)
int16_t motorcal_bd = 0;           // breakaway duty found by this run
int16_t motorcal_vjump = 0;        // peak speed just after breakaway, ADC/s
int16_t motorcal_dwell_a = 0;      // cruise speed at breakaway + A, ADC/s
int32_t motorcal_vel_sum = 0;      // dwell velocity accumulator
uint16_t motorcal_vel_n = 0;
// Per-direction accumulators, indexed [0] = falling, [1] = rising. Integer,
// because flash is the scarce resource on this part and a duty count or an
// ADC count/sec is all the resolution any of these needs.
uint16_t motorcal_bd_sum[2] = {0, 0};
uint16_t motorcal_k_sum[2] = {0, 0};
uint16_t motorcal_vjump_max[2] = {0, 0};
uint8_t motorcal_samples[2] = {0, 0};

bool pending_report_on_idle = false;

bool pending_calibrate_touch = false;
void calibrate_touch();
void reset_tap_detection();

// Forward declarations for layer management functions (Protocol v5+)
void request_layer_change(uint8_t new_layer);
void apply_layer_change(uint8_t new_layer);
void write_layer_target(uint8_t layer, uint8_t target);
void apply_move_speed(uint8_t speed);
void write_layer_haptic_config(uint8_t layer, uint16_t config);

Mode get_mode() {
  return static_cast<Mode>((state & STATE_MODE_bm) >> STATE_MODE_bp);
}

void set_mode(Mode mode) {
  state = (state & ~STATE_MODE_bm) | (mode << STATE_MODE_bp);

  // Any fresh movement starts without accumulated stiction ramp, and with the
  // drive ceiling back down so backlash is taken up gently.
  if (mode != MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
    stiction_ramp = 0;
  } else {
    drive_ceiling = move_takeup_duty;
  }

  // Reset tap detection when entering modes where taps shouldn't be detected
  if (mode != MODE_INPUT_IDLE && mode != MODE_INPUT_ACTIVE) {
    reset_tap_detection();
  }
}

// ============================================================================
// Motor drive
// ============================================================================
// PA4 = Motor A (WO4/HCMP1), PA5 = Motor B (WO5/HCMP2). Both feed the DRV8837
// IN1/IN2 inputs: 1/0 = forward, 0/1 = reverse, 0/0 = coast (Hi-Z), 1/1 = brake.
#define MOTOR_A_bm (1 << 4)
#define MOTOR_B_bm (1 << 5)

enum MotorIdle : uint8_t {
  MOTOR_IDLE_COAST = 0,  // Bridge Hi-Z: fader is free to move
  MOTOR_IDLE_BRAKE = 1,  // Both low sides on: dynamic braking against motion
};

int16_t motor_drive_value = 0;  // Last applied signed drive, for status/LED

// Apply a signed motor drive.
//   drive > 0 pushes toward the motor end (rising ADC), < 0 toward the low end.
//   |drive| is 0..254, matching the TCA0 split-mode period.
//
// slow_decay selects how the off-portion of each PWM cycle is handled:
//   false (fast decay): PWM alternates drive <-> coast. Winding current decays
//     through the body diodes each cycle, so at high PWM frequency the average
//     current is far below what the duty implies and low-duty torque collapses.
//   true (slow decay): PWM alternates drive <-> brake, keeping current
//     circulating through the bridge. Average current (and torque) tracks duty
//     much more linearly, which is what lets us run an inaudible carrier
//     without losing low-speed authority.
void motor_set(int16_t drive, bool slow_decay, MotorIdle idle_mode) {
  if (drive > 254) drive = 254;
  if (drive < -254) drive = -254;
  motor_drive_value = drive;

  if (drive == 0) {
    // Set the static pin levels before releasing the pins from the timer, so
    // handover can't briefly present a drive combination.
    if (idle_mode == MOTOR_IDLE_BRAKE) {
      VPORTA.OUT |= (MOTOR_A_bm | MOTOR_B_bm);
    } else {
      VPORTA.OUT &= ~(MOTOR_A_bm | MOTOR_B_bm);
    }
    TCA0.SPLIT.CTRLB = 0;
    return;
  }

  uint8_t mag = (drive > 0) ? (uint8_t)drive : (uint8_t)(-drive);

  if (!slow_decay) {
    TCA0.SPLIT.HCMP1 = (drive > 0) ? mag : 0;
    TCA0.SPLIT.HCMP2 = (drive > 0) ? 0 : mag;
    TCA0.SPLIT.CTRLB = (TCA_SPLIT_HCMP1EN_bm | TCA_SPLIT_HCMP2EN_bm);
  } else {
    // Hold the leading pin high from PORT and PWM the trailing pin between
    // drive (low) and brake (high), so its high fraction is the inverse duty.
    if (drive > 0) {
      VPORTA.OUT |= MOTOR_A_bm;
      TCA0.SPLIT.HCMP2 = 254 - mag;
      TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP2EN_bm;
    } else {
      VPORTA.OUT |= MOTOR_B_bm;
      TCA0.SPLIT.HCMP1 = 254 - mag;
      TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP1EN_bm;
    }
  }
}

// Configure TCA0 for a single, permanently inaudible PWM carrier.
// Movement used to drop to DIV256 (306 Hz) because at 19.6 kHz with drive/coast
// PWM the motor needed ~190 duty to break away at all. Slow-decay drive (see
// motor_set) restores low-duty torque at the high carrier, so the audible
// carrier is no longer needed anywhere.
void setup_tca0() {
  // TakeOver TCA0 for PWM
  takeOverTCA0();

  // Enable split mode
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
  
  // Configure TCA0 for single-slope PWM
  // HCMPnEN bit: port output register for the corresponding WO[n+3] pin
  // PA4=WO4=HCMP1, PA5=WO5=HCMP2
  TCA0.SPLIT.CTRLB = (TCA_SPLIT_HCMP1EN_bm | TCA_SPLIT_HCMP2EN_bm);
  TCA0.SPLIT.HPER  = 254;
  TCA0.SPLIT.HCMP1 = 0;
  TCA0.SPLIT.HCMP2 = 0;

  TCA0.SPLIT.CTRLA = TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV4_gc;  // 19.6 kHz
}

// Big-endian multi-byte replies. Consolidating these into helpers rather than
// writing the bytes out at each call site is worth real flash on this part -
// every Wire.write() is a call, and the register handler has a lot of them.
void i2c_write_u16(uint16_t v) {
  uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v};
  Wire.write(b, 2);
}

void i2c_write_u32(uint32_t v) {
  uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
  Wire.write(b, 4);
}

// I2C request handler - called when master requests data
// IMPORTANT: This runs in ISR context - only read i2c_ prefixed state!
void onI2cRequest() {
  uint8_t r = current_register;
  if (r == REG_VERSION) {
      Wire.write(I2C_PROTOCOL_VERSION);
  } else if (r == REG_FW_VERSION) {
      i2c_write_u16(FW_VERSION);
  } else if (r == REG_STATE) {
      // Snapshot once: the main loop can update i2c_outgoing_state between
      // byte writes, which would hand the controller a torn value.
      i2c_write_u32(i2c_outgoing_state);
  } else if (r == REG_UPTIME) {
      i2c_write_u32(millis());
  } else if (r == REG_TOUCH_RAW) {
      i2c_write_u16(touch_sensor.sensorData);
  } else if (r == REG_SERIAL) {
      // Read 10-byte serial number from SIGROW
      Wire.write((const uint8_t *)&SIGROW.SERNUM0, 10);
  } else if (r == REG_TOUCH_DELTA) {
      // Touch delta (signed 16-bit): sensorData - reference
      i2c_write_u16((uint16_t)ptc_get_node_delta(&touch_sensor));
  } else if (r == REG_TOUCH_REF) {
      // Touch reference value (unsigned 16-bit)
      i2c_write_u16(touch_sensor.reference);
  } else if (r == REG_TOUCH_RECAL) {
      // Touch recalibration count (unsigned 16-bit)
      i2c_write_u16(touch_recal_count);
#if DEBUG_DRIVE
  } else if (r == REG_DEBUG_STATUS) {
      int16_t vel = (int16_t)debug_status_velocity;
      int16_t err_x8 = (int16_t)(debug_status_error * 8);
      const uint16_t vals[9] = {
        input_calib_min, input_calib_max, (uint16_t)target_adc,
        (uint16_t)motor_drive_value, (uint16_t)vel, (uint16_t)err_x8,
        debug_status_loop_hz, debug_status_tick_hz,
        (uint16_t)move_max_velocity,
      };
      for (uint8_t i = 0; i < 9; i++) i2c_write_u16(vals[i]);
#endif
  } else if (r == REG_MOTOR_CAL) {
      const uint8_t vals[12] = {
        motor_cal.valid,
        motor_cal.bd[MOTORCAL_RISING], motor_cal.bd[MOTORCAL_FALLING],
        motor_cal.k[MOTORCAL_RISING], motor_cal.k[MOTORCAL_FALLING],
        (uint8_t)(motor_cal.vjump[MOTORCAL_RISING] >> 8),
        (uint8_t)motor_cal.vjump[MOTORCAL_RISING],
        (uint8_t)(motor_cal.vjump[MOTORCAL_FALLING] >> 8),
        (uint8_t)motor_cal.vjump[MOTORCAL_FALLING],
        // What the control law actually runs with, derived or default, so a
        // host can see the outcome without knowing how it is derived.
        (uint8_t)(((uint16_t)move_vel_min) >> 8), (uint8_t)move_vel_min,
        (uint8_t)move_deadband,
      };
      Wire.write(vals, 12);
  } else if (r == REG_ACTIVE_LAYER) {
      Wire.write(active_layer);
  } else if (r == REG_LAYER_TARGET) {
      // Return restore position for the previously queried layer
      Wire.write(layer_restore_positions[queried_layer]);
  } else if (r == REG_LAYER_HAPTIC_CONFIG) {
      // Return haptic config for the previously queried layer (16 bits, big-endian)
      uint16_t config = layer_haptic_configs[queried_layer];
      Wire.write((config >> 8) & 0xFF);  // High byte
      Wire.write(config & 0xFF);          // Low byte
  }
}

// I2C receive handler - called when master sends data
// IMPORTANT: This runs in ISR context - only set flags/copy data, no state changes!
void onI2cReceive(int howMany) {
  if (howMany == 0) return;

  // First byte is always the register address
  current_register = Wire.read();

  switch (current_register) {
    case REG_CAL_TOUCH:
      pending_calibrate_touch = true;
      break;
    case REG_CLEAR_ERROR:
      // Set flag for main loop to process
      i2c_clear_error_request = true;
      break;
    case REG_SELF_CAL:
      // Set flag for main loop to process
      i2c_self_cal_request = true;
      break;
    case REG_ACTIVE_LAYER:
      if (howMany == 2) {  // register + 1 byte layer index
        uint8_t new_layer = Wire.read() & 0x07;  // Clamp to 0-7
        i2c_layer_change_request = new_layer;
      }
      break;
    case REG_LAYER_TARGET:
      if (howMany == 2) {
        // Read setup: register + layer index
        queried_layer = Wire.read() & 0x07;
      } else if (howMany == 3 || howMany == 4) {
        // Write: register + layer + target position [+ optional speed].
        // The 3-byte form is the original protocol and leaves the layer's
        // existing speed alone, so old controllers are unaffected.
        i2c_layer_target_write.layer = Wire.read() & 0x07;
        i2c_layer_target_write.target = Wire.read();
        i2c_layer_target_write.has_speed = (howMany == 4);
        i2c_layer_target_write.speed = (howMany == 4) ? Wire.read() : LAYER_SPEED_FULL;
        i2c_layer_target_write.valid = true;
      }
      break;
    case REG_LAYER_HAPTIC_CONFIG:
      if (howMany == 2) {
        // Read setup: register + layer index
        queried_layer = Wire.read() & 0x07;
      } else if (howMany == 4) {
        // Write: register + layer + 2 bytes config (big-endian)
        // Copy data to volatile struct for main loop to process
        i2c_layer_haptic_write.layer = Wire.read() & 0x07;
        i2c_layer_haptic_write.config = ((uint16_t)Wire.read() << 8) | Wire.read();
        i2c_layer_haptic_write.valid = true;
      }
      break;
#if DEBUG_DRIVE
    case REG_DEBUG_GAINS:
      if (howMany == 4) {
        i2c_debug_gain_index = Wire.read();
        i2c_debug_gain_value = (int16_t)(((uint16_t)Wire.read() << 8) | Wire.read());
        i2c_debug_gain_valid = true;
      }
      break;
    case REG_DEBUG_DRIVE:
      if (howMany == 3) {
        i2c_debug_drive_flags = Wire.read();
        i2c_debug_drive_duty = Wire.read();
        i2c_debug_drive_valid = true;
      }
      break;
#endif
    case REG_VERSION:
    case REG_FW_VERSION:
    case REG_STATE:
    case REG_UPTIME:
    case REG_TOUCH_RAW:
    case REG_SERIAL:
    case REG_TOUCH_DELTA:
    case REG_TOUCH_REF:
    case REG_TOUCH_RECAL:
      // Read-only registers, ignore writes
      // Discard any excess data
      while (Wire.available()) Wire.read();
      break;

    default:
      // Unknown register, discard data
      // Discard any excess data
      while (Wire.available()) Wire.read();
      break;
  }

}

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

// Checksum over everything in the record except the checksum itself.
uint16_t calibration_checksum(const CalibrationData &cal) {
  uint16_t sum = cal.magic + cal.calib_min + cal.calib_max + cal.motor.valid;
  for (uint8_t i = 0; i < 2; i++) {
    sum += cal.motor.bd[i] + cal.motor.k[i] + cal.motor.vjump[i];
  }
  return sum;
}

// Save calibration data to EEPROM
void saveCalibration() {
  CalibrationData cal;
  cal.magic = EEPROM_CALIBRATION_MAGIC;
  cal.calib_min = input_calib_min;
  cal.calib_max = input_calib_max;
  cal.motor = motor_cal;
  cal.checksum = calibration_checksum(cal);

  EEPROM.put(EEPROM_CALIBRATION_ADDR, cal);
}

// Load calibration data from EEPROM
// Returns true if valid calibration was loaded, false otherwise
bool loadCalibration() {
  CalibrationData cal;
  EEPROM.get(EEPROM_CALIBRATION_ADDR, cal);

  // Validate magic number
  if (cal.magic != EEPROM_CALIBRATION_MAGIC) {
    return false;
  }

  // Validate checksum
  if (cal.checksum != calibration_checksum(cal)) {
    return false;
  }

  // Validate calibration values are reasonable
  if (cal.calib_min >= cal.calib_max) {
    return false;
  }

  if (cal.calib_max - cal.calib_min < 900) {
    // Span is too small, likely invalid
    return false;
  }

  // Load calibration values
  input_calib_min = cal.calib_min;
  input_calib_max = cal.calib_max;
  motor_cal = cal.motor;

  return true;
}

void setup_i2c() {
  // Straight to the port. The three address jumpers are PC0-PC2, read once at
  // boot, and going through pinMode()/digitalRead() for them links both
  // functions (~120 bytes) for no benefit.
  PORTC.PIN0CTRL |= PORT_PULLUPEN_bm;
  PORTC.PIN1CTRL |= PORT_PULLUPEN_bm;
  PORTC.PIN2CTRL |= PORT_PULLUPEN_bm;
  _delay_us(10);  // let the pullups settle before sampling

  // A fitted jumper pulls its pin low, so the bits are inverted. Note the
  // address pins run the opposite way round from the port bits: PIN_ADDR_0 is
  // PC2 and PIN_ADDR_2 is PC0, so the three bits are reversed, not copied.
  uint8_t fitted = (~VPORTC.IN) & 0x07;  // bit n set = jumper on PCn
  uint8_t address = I2C_BASE_ADDRESS +
    ((fitted & (1 << 0)) << 2) +  // PC0 -> A2
    ((fitted & (1 << 1))) +       // PC1 -> A1
    ((fitted & (1 << 2)) >> 2);   // PC2 -> A0

  // Initialize I2C as slave
  Wire.begin(address);
  Wire.onRequest(onI2cRequest);
  Wire.onReceive(onI2cReceive);
}

void increment_position_nonce() {
  uint8_t position_nonce = (state & STATE_POSITION_NONCE_bm) >> STATE_POSITION_NONCE_bp;
  position_nonce++;
  position_nonce &= ((1 << STATE_POSITION_NONCE_bs) - 1);
  state &= ~STATE_POSITION_NONCE_bm;
  state |= ((uint32_t)position_nonce << STATE_POSITION_NONCE_bp);// & STATE_POSITION_NONCE_bm;
}

void increment_double_tap_nonce() {
  double_tap_nonce++;
  double_tap_nonce &= ((1 << STATE_DOUBLE_TAP_NONCE_bs) - 1);
  state &= ~STATE_DOUBLE_TAP_NONCE_bm;
  state |= ((uint32_t)double_tap_nonce << STATE_DOUBLE_TAP_NONCE_bp);// & STATE_DOUBLE_TAP_NONCE_bm;
}

void reset_tap_detection() {
  tap_state = TAP_NONE;
}

// How far the carriage has moved since the tap started. Raw ADC deliberately,
// not input_ewma: the filter's lag is comparable to the tap timings being
// judged here, so a filtered value would report movement that already ended.
uint16_t tap_position_delta() {
  uint16_t current_adc = ADC1.RES;
  return (current_adc > tap_position_start) ? (current_adc - tap_position_start)
                                            : (tap_position_start - current_adc);
}

// Calculate max PWM from 3-bit strength value (0-7)
// Returns maximum PWM value to use for haptic force
// strength 0 -> minimum usable PWM (~189), strength 7 -> full PWM limit (254)
uint8_t get_strength_max_pwm(uint8_t strength) {
  // Scale from 189 (minimum usable) to 254 (max) based on strength
  // strength 0: 189 max PWM, strength 7: 254 max PWM
  const uint8_t min_pwm = 189;  // Minimum usable PWM (was strength 2)
  return min_pwm + (strength * (HAPTIC_MAX_PWM - min_pwm)) / 7;
}

// Calculate the nearest detent position in ADC units
// detent_count: number of detents (1-10)
// current_position: current fader position in ADC units
// Returns: ADC value of the nearest detent
uint16_t get_nearest_detent_position(uint8_t detent_count, uint16_t current_position) {
  // Special case: single detent at midpoint
  if (detent_count == 1) {
    return input_calib_min + (input_calib_max - input_calib_min) / 2;
  }

  // For 2+ detents: evenly spaced including endpoints
  uint16_t range = input_calib_max - input_calib_min;

  // Calculate offset from minimum position
  int16_t offset = current_position - input_calib_min;

  // Calculate which detent index is nearest using rounding
  // detent_index = round(offset * (detent_count - 1) / range)
  // Using integer math: add half the divisor before dividing for rounding
  uint8_t detent_index = ((uint32_t)offset * (detent_count - 1) + range / 2) / range;

  // Clamp to valid range [0, detent_count-1]
  if (detent_index >= detent_count) {
    detent_index = detent_count - 1;
  }

  // Calculate position of that detent
  return input_calib_min + ((uint32_t)detent_index * range) / (detent_count - 1);
}

// ============================================================================
// Layer Management Functions (Protocol v5+)
// ============================================================================

// Request layer change - may be deferred based on current mode
void request_layer_change(uint8_t new_layer) {
  if (new_layer > 7) return;  // Invalid layer
  if (new_layer == active_layer) return;  // Already on this layer

  Mode mode = get_mode();
  if (mode == MODE_ERROR) {
    return;  // Ignore - no deferral
  }

  pending_layer_change = new_layer;  // Defer until appropriate to apply
}

// Enter (or restart) a remote move toward the current target_adc. Every caller
// needs the same three timestamps reset together - a half-reset move either
// times out early or never does.
void begin_remote_move() {
  uint32_t now = millis();
  remote_movement_start = now;
  remote_movement_start_position = input_ewma;
  remote_movement_steady_start = now;
  set_mode(MODE_REMOTE_MOVEMENT_IN_PROGRESS);
}

// Apply layer change and start movement to new layer's restore position
void apply_layer_change(uint8_t new_layer) {
  // Start movement to new layer's restore position
  target_adc = BOUNDED_LERP_UINT16(
    layer_restore_positions[new_layer], 0, 255,
    input_calib_min, input_calib_max
  );
  apply_move_speed(layer_speeds[new_layer]);
  begin_remote_move();

  // Load new layer's haptic config
  haptic_config = layer_haptic_configs[new_layer];

  pending_layer_change = 0xFF;
  active_layer = new_layer;
}

// Write target position to a specific layer
void write_layer_target(uint8_t layer, uint8_t target) {
  if (layer > 7) return;

  if (layer == active_layer) {
    // Starting a move and retargeting one already under way are the same
    // thing. Anything else - the user has control, an error is latched, a
    // calibration is running - leaves the fader alone.
    Mode mode = get_mode();
    if (mode == MODE_INPUT_IDLE || mode == MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
      layer_restore_positions[layer] = target;
      target_adc = BOUNDED_LERP_UINT16(target, 0, 255, input_calib_min, input_calib_max);
      apply_move_speed(layer_speeds[layer]);
      begin_remote_move();
    }
  } else {
    // Non-active layer - just update restore position
    layer_restore_positions[layer] = target;
  }
}

// Write haptic configuration to a specific layer (16-bit format)
void write_layer_haptic_config(uint8_t layer, uint16_t config) {
  if (layer > 7) return;

  // Validate haptic configuration
  HapticMode haptic_mode = static_cast<HapticMode>((config & HAPTIC_MODE_bm) >> HAPTIC_MODE_bp);
  if (haptic_mode == HAPTIC_DETENTS) {
    uint8_t detent_count = (config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp;
    if (detent_count < 1 || detent_count > 10) {
      return;  // Invalid config - reject
    }
  }

  // Store config for layer
  layer_haptic_configs[layer] = config;

  // If active layer, apply immediately (even during MODE_INPUT_ACTIVE)
  if (layer == active_layer) {
    haptic_config = config;
  }
}

// Fold every completed free-running ADC1 conversion into the position filter.
// Draining on the RESRDY flag rather than sampling RES whenever the main loop
// happens to come around means the filter runs at the ADC's own constant rate
// and never re-uses or tears a result.
uint16_t adc_last_raw = 0;
void adc_drain() {
  while (ADC1.INTFLAGS & ADC_RESRDY_bm) {
    uint16_t adc_val = ADC1.RES;  // reading RES clears RESRDY
    adc_last_raw = adc_val;
    // 2x accumulation, so halve to get ADC counts
    input_ewma = adc_val * ALPHA / 2 + input_ewma * (1 - ALPHA);
  }
}

// Translate a layer's unitless speed byte into the velocity limit used by the
// control law. LAYER_SPEED_FULL means no limit; below that the byte maps
// linearly in VELOCITY (not in move time) across the validated window, so the
// host-facing scale is a speed dial and equal steps feel like equal changes.
// The window is expressed as full-travel times, so the endpoints are stated in
// the same terms the measurements were taken in; the calibrated span converts
// each to a velocity.
void apply_move_speed(uint8_t speed) {
  if (speed >= LAYER_SPEED_FULL) {
    move_max_velocity = MOVE_VEL_UNLIMITED;
    return;
  }
  // Integer throughout: the result is an ADC-counts/sec limit that the plant
  // only honours to within ~15% anyway, so sub-count precision here would buy
  // nothing and float on this part is not free.
  uint16_t span = input_calib_max - input_calib_min;
  uint16_t slowest = ((uint32_t)span * 1000) / MOVE_SPEED_SLOWEST_MS;
  // The fast endpoint divides 1000 exactly, so it stays in 16-bit arithmetic
  // (span is at most 1023, so span * 4 cannot overflow). The static_asserts
  // keep that true if the endpoint is ever re-measured and changed.
  static_assert(1000 % MOVE_SPEED_FASTEST_MS == 0,
                "MOVE_SPEED_FASTEST_MS must divide 1000 exactly, or use 32-bit math here");
  static_assert(1023UL * (1000 / MOVE_SPEED_FASTEST_MS) <= 65535UL, "span scaling overflows");
  uint16_t fastest = span * (1000 / MOVE_SPEED_FASTEST_MS);
  uint16_t adc_per_sec =
      slowest + (uint16_t)(((uint32_t)(fastest - slowest) * speed) / (LAYER_SPEED_FULL - 1));
  // Clamp to what the mechanism can actually sustain smoothly
  if (adc_per_sec < move_vel_min) adc_per_sec = move_vel_min;
  move_max_velocity = adc_per_sec;
}

// Direction of travel for the current characterisation run: runs alternate,
// starting with falling, so each begins where the previous one ended.
int8_t motorcal_dir() {
  return (motorcal_run & 1) ? 1 : -1;
}

// A run drives open-loop, so it has to police its own travel. Bail out before
// the carriage reaches the mechanical end rather than measuring a fader that
// is being held by an endstop.
bool motorcal_out_of_band(int8_t dir) {
  if (dir > 0) return input_ewma > input_calib_max - MOTORCAL_EDGE_MARGIN;
  return input_ewma < input_calib_min + MOTORCAL_EDGE_MARGIN;
}

void motor_update() {
  uint32_t now = millis();

  uint16_t adc_val = adc_last_raw;

  // Velocity for the damping term: differentiate the filtered position, then
  // smooth with a fixed time constant. Dividing by the measured interval
  // rather than the nominal tick keeps the estimate correct even when a tick
  // lands late, and input_ewma is a float fed by a dithered ADC, so its
  // differences stay meaningful well below one ADC count.
  float dt = control_dt_us * 1e-6f;
  if (dt > 0.0f) {
    float dv = (input_ewma - last_control_ewma) / dt;
    float alpha_v = dt / VELOCITY_TAU_S;
    if (alpha_v > 1.0f) alpha_v = 1.0f;
    velocity_ewma += alpha_v * (dv - velocity_ewma);
  }
  last_control_ewma = input_ewma;

#if DEBUG_DRIVE
  debug_status_velocity = velocity_ewma;
  debug_status_error = target_adc - input_ewma;
  debug_tick_count++;
  if (now - debug_rate_window_start >= 500) {
    debug_status_loop_hz = debug_loop_count * 2;
    debug_status_tick_hz = debug_tick_count * 2;
    debug_loop_count = 0;
    debug_tick_count = 0;
    debug_rate_window_start = now;
  }
#endif

  Mode mode = get_mode();

  // If we didn't get a second tap start in time, reset tap detection
  if (tap_state == TAP_WAITING_FOR_DOUBLE) {
    if (now - tap_timestamp > DOUBLE_TAP_MAX_INTERVAL) {
      reset_tap_detection();
    }
  }

  if (input_ewma > position_window_upper) {
    position_window_upper = input_ewma;
    position_window_lower = position_window_upper - WINDOW_SIZE;
    if (mode != MODE_REMOTE_MOVEMENT_IN_PROGRESS && position != position_window_upper) {
      input_last_change_millis = now;
    }
    position = position_window_upper;
  } else if (input_ewma < position_window_lower) {
    position_window_lower = input_ewma;
    position_window_upper = position_window_lower + WINDOW_SIZE;
    if (mode != MODE_REMOTE_MOVEMENT_IN_PROGRESS && position != position_window_lower) {
      input_last_change_millis = now;
    }
    position = position_window_lower;
  }

#if DEBUG_DRIVE
  if (debug_drive_active) {
    int16_t d = debug_drive_value;
    if (now - debug_drive_last_update > DEBUG_DRIVE_WATCHDOG_MS) {
      // Host stopped refreshing - never leave the motor driven unattended
      debug_drive_active = false;
      d = 0;
    } else if ((d > 0 && input_ewma > input_calib_max - DEBUG_DRIVE_EDGE_MARGIN) ||
               (d < 0 && input_ewma < input_calib_min + DEBUG_DRIVE_EDGE_MARGIN)) {
      // Don't drive into the endstops
      d = 0;
    }
    motor_set(d, debug_drive_slow_decay,
              debug_drive_brake ? MOTOR_IDLE_BRAKE : MOTOR_IDLE_COAST);
  } else
#endif
  switch (mode) {
    case MODE_REMOTE_MOVEMENT_IN_PROGRESS:
      if (now > remote_movement_start + MOVEMENT_TIMEOUT_MILLIS) {
        // Distinguish a real fault (jam, dead motor, unreachable target) from a
        // fader that arrived but kept dithering across the deadband. The latter
        // is a tuning mismatch, not a failure, and reporting it as an error
        // leaves the fader dead until the host clears it.
        float timeout_error = target_adc - input_ewma;
        if (timeout_error < 0) timeout_error = -timeout_error;
        if (timeout_error <= MOVE_TIMEOUT_TOLERANCE) {
          motor_set(0, false, MOTOR_IDLE_COAST);
          input_last_change_millis = now - IDLE_DURATION_THRESHOLD;
          set_mode(Mode::MODE_INPUT_IDLE);
        } else {
          set_mode(Mode::MODE_ERROR);
        }
      } else if ((state & STATE_TOUCH_bm) && now > touch_state_change_millis + TOUCH_OVERRIDE_DURATION_THRESHOLD) {
        set_mode(Mode::MODE_INPUT_ACTIVE);
      } else {
        // Apply pending layer change
        if (pending_layer_change != 0xFF) {
          apply_layer_change(pending_layer_change);
          break;  // Exit switch since state may change
        }
        float error = target_adc - input_ewma;
        if (error > move_deadband || error < -move_deadband) {
          // Cascade control: the position loop turns error into a velocity
          // reference, the velocity loop delivers it via feedforward (plant
          // inverted) plus feedback. See "The control law" in
          // ABOUT_MOTOR_CONTROL.md for the derivation.
          float v_ref = move_vref_slope * error;
          float mag = v_ref < 0 ? -v_ref : v_ref;
          if (move_max_velocity > MOVE_VEL_UNLIMITED && mag > move_max_velocity) {
            mag = move_max_velocity;
          }
          // Never ask for less than the mechanism can actually sustain: below
          // the Stribeck floor it stick-slips rather than moving, and holding
          // the reference here is also what keeps the feedforward clear of
          // breakaway when the error is small.
          if (mag < move_vel_min) mag = move_vel_min;

          uint8_t d = (error > 0) ? MOTORCAL_RISING : MOTORCAL_FALLING;
          float ff = move_bd[d] + mag * move_inv_k[d];
          if (error < 0) ff = -ff;
          v_ref = (error > 0) ? mag : -mag;
          float u = ff + move_kv * (v_ref - velocity_ewma);

          // Ramp in extra drive only while stalled. The feedforward is derived
          // to sit above breakaway, so this should now be rare - it covers
          // measurement error, a cold or stiff unit, and the uncharacterised
          // case.
          float speed = velocity_ewma < 0 ? -velocity_ewma : velocity_ewma;
          if (speed < MOVE_STALL_VELOCITY) {
            stiction_ramp += move_ramp_rate * (control_dt_us * 1e-6f);
            if (stiction_ramp > MOVE_RAMP_MAX) stiction_ramp = MOVE_RAMP_MAX;
          } else if (stiction_ramp > 0) {
            // Shed it quickly once the carriage breaks free, but not in one
            // step: dropping up to MOVE_RAMP_MAX of drive instantly is itself a
            // relay, and around the stall threshold it chatters. A few tens of
            // ms is far shorter than the coast time, so it still cannot
            // contribute to overshoot.
            stiction_ramp -= MOVE_RAMP_DECAY_RATE * (control_dt_us * 1e-6f);
            if (stiction_ramp < 0) stiction_ramp = 0;
          }
          u += (error > 0) ? stiction_ramp : -stiction_ramp;

          // Drive ceiling opens up over time from the take-up value, easing
          // the motor through belt backlash instead of stepping to full duty.
          drive_ceiling += move_takeup_ramp_rate * (control_dt_us * 1e-6f);
          if (drive_ceiling > MOVE_MAX_DUTY) drive_ceiling = MOVE_MAX_DUTY;
          int16_t limit = (int16_t)drive_ceiling + (int16_t)stiction_ramp;
          if (limit > MOVE_MAX_DUTY) limit = MOVE_MAX_DUTY;

          int16_t drive = (int16_t)u;
          if (drive > limit) drive = limit;
          if (drive < -limit) drive = -limit;
          motor_set(drive, true, MOTOR_IDLE_BRAKE);
          remote_movement_steady_start = now;
        } else {
          // On target: hold with the bridge braked until we declare the move
          // finished, so a loose carriage can't drift back out of the window.
          stiction_ramp = 0;
          motor_set(0, true, MOTOR_IDLE_BRAKE);
          if (now > remote_movement_steady_start + REMOTE_MOVEMENT_STEADY_THRESHOLD) {
            // shift hysteresis window to prevent spurious immediate "input" detection if remote movement left us near the window bounds and succeptiple to noise
            if (input_ewma < WINDOW_SIZE / 2) {
              position_window_lower = 0;
              position_window_upper = WINDOW_SIZE;
            } else if (input_ewma > 1023 - WINDOW_SIZE / 2) {
              position_window_upper = 1023;
              position_window_lower = 1023 - WINDOW_SIZE / 2;
            } else {
              position_window_lower = input_ewma - WINDOW_SIZE / 2;
              position_window_upper = position_window_lower + WINDOW_SIZE;
            }
            input_last_change_millis = now - IDLE_DURATION_THRESHOLD;
            set_mode(Mode::MODE_INPUT_IDLE);
          }
        }
      }
      break;
    case MODE_INPUT_ACTIVE:
      // Continuously update active layer's restore position
      {
        uint8_t current_pos = BOUNDED_LERP_UINT16(position, input_calib_min, input_calib_max, 0, 255);
        layer_restore_positions[active_layer] = current_pos;
      }

      if (now > input_last_change_millis + IDLE_DURATION_THRESHOLD && (state & STATE_TOUCH_bm) == 0 && now > touch_state_change_millis + IDLE_DURATION_THRESHOLD) {
        motor_set(0, false, MOTOR_IDLE_COAST);
        if (pending_report_on_idle) {
          pending_report_on_idle = false;
          increment_position_nonce();
        }
        set_mode(Mode::MODE_INPUT_IDLE);
      } else {
        // Haptics - extract current mode from haptic_config
        HapticMode haptic_mode = static_cast<HapticMode>((haptic_config & HAPTIC_MODE_bm) >> HAPTIC_MODE_bp);

        if (haptic_mode == HAPTIC_SMOOTH_WITH_MAGNET_ENDS) {
          // Magnetic endpoints - pull toward calibration limits when near
          uint8_t strength = (haptic_config & HAPTIC_DETENT_STRENGTH_bm) >> HAPTIC_DETENT_STRENGTH_bp;
          uint8_t max_pwm = get_strength_max_pwm(strength);

          if (input_ewma < input_calib_min + HAPTIC_MAGNET_RANGE && input_ewma > input_calib_min + HAPTIC_DEAD_ZONE) {
            float delta = (input_calib_min - input_ewma) * HAPTIC_BASE_MULTIPLIER;
            uint8_t pwm = (-delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : -delta + HAPTIC_BASE_PWM;
            motor_set(-(int16_t)pwm, false, MOTOR_IDLE_COAST);
          } else if (input_ewma > input_calib_max - HAPTIC_MAGNET_RANGE && input_ewma < input_calib_max - HAPTIC_DEAD_ZONE) {
            float delta = (input_calib_max - input_ewma) * HAPTIC_BASE_MULTIPLIER;
            uint8_t pwm = (delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : delta + HAPTIC_BASE_PWM;
            motor_set((int16_t)pwm, false, MOTOR_IDLE_COAST);
          } else {
            motor_set(0, false, MOTOR_IDLE_COAST);
          }
        } else if (haptic_mode == HAPTIC_DETENTS) {
          // Detent haptics - pull toward nearest detent position
          uint8_t detent_count = (haptic_config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp;
          uint8_t strength = (haptic_config & HAPTIC_DETENT_STRENGTH_bm) >> HAPTIC_DETENT_STRENGTH_bp;
          uint8_t max_pwm = get_strength_max_pwm(strength);

          // Get nearest detent position
          uint16_t nearest_detent = get_nearest_detent_position(detent_count, input_ewma);

          // Calculate displacement from detent (positive = need to move up, negative = need to move down)
          int16_t displacement = nearest_detent - input_ewma;

          // Apply dead zone
          if (abs(displacement) > HAPTIC_DEAD_ZONE) {
            // Calculate restorative force proportional to displacement
            float delta = displacement * HAPTIC_BASE_MULTIPLIER;

            if (delta > 0) {
              // Pull toward higher position (Motor A)
              uint8_t pwm = (delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : delta + HAPTIC_BASE_PWM;
              motor_set((int16_t)pwm, false, MOTOR_IDLE_COAST);
            } else {
              // Pull toward lower position (Motor B)
              uint8_t pwm = (-delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : -delta + HAPTIC_BASE_PWM;
              motor_set(-(int16_t)pwm, false, MOTOR_IDLE_COAST);
            }
          } else {
            // Within dead zone, no force
            motor_set(0, false, MOTOR_IDLE_COAST);
          }
        } else {
          // No haptics for NO_HAPTICS mode
          motor_set(0, false, MOTOR_IDLE_COAST);
        }
      }
      break;
    case MODE_INPUT_IDLE:
      motor_set(0, false, MOTOR_IDLE_COAST);

      // Apply pending layer change
      if (pending_layer_change != 0xFF) {
        apply_layer_change(pending_layer_change);
        break;  // Exit switch since state may change
      }

      if (now < input_last_change_millis + IDLE_DURATION_THRESHOLD || ((state & STATE_TOUCH_bm) != 0 && now > touch_state_change_millis + TOUCH_OVERRIDE_DURATION_THRESHOLD)) {
        set_mode(Mode::MODE_INPUT_ACTIVE);
      }
      break;
    case MODE_ERROR:
      motor_set(0, false, MOTOR_IDLE_COAST);
      break;
    case MODE_SELF_CALIBRATION: {
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
            self_calibration_adc_stage_0 = adc_val;
            self_calibration_stage++;
            self_calibration_start = millis();
          } else {
            // Move toward lower ADC value
            motor_set(-254, true, MOTOR_IDLE_COAST);
          }
          break;
        case SELFCAL_ENDPOINT_HIGH:
          if (now > self_calibration_start + SELF_CALIBRATION_TIMEOUT) {
            self_calibration_adc_stage_1 = adc_val;
            self_calibration_stage++;
            self_calibration_start = millis();
          } else {
            // Move toward higher ADC value
            motor_set(254, true, MOTOR_IDLE_COAST);
          }
          break;
        case SELFCAL_ENDPOINT_APPLY:
          motor_set(0, false, MOTOR_IDLE_COAST);
          if (abs((int16_t)self_calibration_adc_stage_0 - self_calibration_adc_stage_1) < 900) {
            set_mode(Mode::MODE_ERROR);
          } else {
            // Apply calibration in memory
            // Divide by 2 to account for 2-sample ADC aggregation (adc_val is 2x, but input_ewma is corrected)
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
            motor_set(0, true, MOTOR_IDLE_BRAKE);
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
          motor_set(0, true, MOTOR_IDLE_BRAKE);
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
            // Still hunting for breakaway. Fixed point, 1/256 duty per LSB:
            // the ramp is the only thing here needing sub-duty resolution, and
            // a float multiply per tick for it is not worth the flash. The
            // fraction has to be fine enough that a tick's increment does not
            // truncate to nothing - 84/4096 per us is 80.1 duty/sec at the
            // 500 us tick, where a coarser LSB would quietly give ~62.
            motorcal_duty += (uint16_t)((control_dt_us * 84UL) >> 12);
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
          motor_set(0, true, MOTOR_IDLE_BRAKE);
          motorcal_run++;
          self_calibration_stage =
              (!motorcal_failed && motorcal_run < 2 * MOTORCAL_PASSES)
                  ? SELFCAL_MOTOR_PARK
                  : SELFCAL_FINISH;
          self_calibration_start = now;
          break;
        case SELFCAL_FINISH:
          motor_set(0, false, MOTOR_IDLE_COAST);
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

          // Save calibration to EEPROM
          saveCalibration();

          // Since we moved the position, do a remote movement to the previous target
          // TODO: re-calculate target_adc using the new calibration bounds. Can't do that now since we lerp target to an ADC value upon receipt, without saving the 0-255 value
          begin_remote_move();
          break;
      }
      break;
    }
    }

  // TODO: lerp bounds...
  uint8_t pos = BOUNDED_LERP_UINT16(position, input_calib_min, input_calib_max, 0, 255);
  state &= ~STATE_POSITION_bm;
  state |= pos << STATE_POSITION_bp;

  state &= ~(uint32_t)STATE_RAW_ADC_bm;
  state |= ((uint32_t)adc_val << STATE_RAW_ADC_bp) & (uint32_t)STATE_RAW_ADC_bm;

  // Pack active layer into state (bits 4-6, replacing haptic_config_nonce in v5)
  state &= ~STATE_ACTIVE_LAYER_bm;
  state |= ((uint32_t)active_layer << STATE_ACTIVE_LAYER_bp) & STATE_ACTIVE_LAYER_bm;

#if SERIAL_ENABLED
  static uint32_t last_print;
  if (now - last_print > 200) {
    last_print = now;
    Serial.println("");
    Serial.println(delta, DEC);
    Serial.println(get_position(), DEC);
  }
#endif
}

void setup_touch() {
  ptc_add_selfcap_node(&touch_sensor, PIN_TO_PTC(PIN_TOUCH), 0);
  ptc_node_set_thresholds(&touch_sensor, 200, 50);
  ptc_node_set_gain(&touch_sensor, ADC_SAMPNUM_ACC1_gc, ADC_SAMPNUM_ACC1_gc);

  ptc_lib_sm_set_t* settings = ptc_get_sm_settings();
  settings->drift_down_nom = 50;
  settings->drift_up_nom = 50;
  settings->force_recal_delta = 200; // Increase drift recalibration delta?
  settings->touched_max_nom = 255; // Disable automatic recalibration on very long touch
  settings->touched_detect_nom = 2;
  settings->untouched_detect_nom = 2;
}

void setup() {
#if SERIAL_ENABLED
  Serial.swap(1); // RX/TX on alternate pins
  Serial.begin(115200);
  Serial.println("Hello world!");
#endif

  // Load calibration from EEPROM if available; defaults stand if there is none.
  bool have_calibration = loadCalibration();
  // Unconditional, and after the load either way: the derived gains have no
  // compile-time initialiser, so a unit with no stored calibration still has to
  // run the derivation to pick up the default plant model.
  apply_motor_calibration();
#if SERIAL_ENABLED
  if (have_calibration) {
    Serial.print("Loaded calibration from EEPROM: min=");
    Serial.print(input_calib_min);
    Serial.print(", max=");
    Serial.println(input_calib_max);
  } else {
    Serial.println("No valid calibration found, using defaults");
  }
#else
  (void)have_calibration;
#endif

  // Initialize all layers with default configuration (Protocol v5+)
  for (uint8_t i = 0; i < 8; i++) {
    layer_haptic_configs[i] = 0;  // Default: HAPTIC_NO_HAPTICS (smooth mode), all bits 0
    layer_restore_positions[i] = 128;  // Default: midpoint
    layer_speeds[i] = LAYER_SPEED_FULL;
  }
  active_layer = 0;
  pending_layer_change = 0xFF;  // No pending change
  queried_layer = 0;

  // Load active layer's haptic config into global haptic_config
  haptic_config = layer_haptic_configs[0];

  setup_i2c();

  // Direct port setup, for the same reason as the address pins above.
  // PB2 LED, PB3 nSLEEP out; PA4/PA5 motor out; PA6 fader in.
  VPORTB.DIR |= LED_bm | MOTOR_nSLEEP_bm;
  VPORTA.DIR |= MOTOR_A_bm | MOTOR_B_bm;
  VPORTA.DIR &= ~(1 << 6);

  // Set up TCA0 for high-frequency PWM
  setup_tca0();
  
  // Direct to the port, as in loop(): these two calls are the only reason
  // digitalWrite() would be linked in at all, and it costs 160 bytes.
  VPORTB.OUT |= MOTOR_nSLEEP_bm | LED_bm;

  // Init ADC1 for free-running motor fader input.
  // Using ADC1 with raw setup rather than megaTinyCore's analogRead helpers since
  // we need to leave ADC0 free for the PTC touch library.
  init_ADC1();
  ADC1.MUXPOS=0x02; //reads from PA6, ADC1 channel 2
  ADC1.CTRLB = ADC_SAMPNUM_ACC2_gc; // Accumulate 2 readings
  ADC1.CTRLA=ADC_ENABLE_bm|ADC_FREERUN_bm; //start in freerun
  ADC1.COMMAND=ADC_STCONV_bm; //start first conversion!

  // Prime the position filter from a real conversion, so the first control
  // tick doesn't see a huge phantom velocity as the filter slews up from zero.
  while (!(ADC1.INTFLAGS & ADC_RESRDY_bm)) { }
  adc_last_raw = ADC1.RES;
  input_ewma = adc_last_raw / 2.0f;
  last_control_ewma = input_ewma;
  last_control_tick_us = micros();
  last_control_exec_us = last_control_tick_us;

  setup_touch();
  pending_calibrate_touch = true;
}

// Process I2C requests that were queued by ISR callbacks
// This must be called from main loop (non-ISR context) before motor_update()
void process_i2c_requests() {
  // Local copies of all i2c requests
  bool clear_error = false;
  bool self_cal = false;
  uint8_t layer_change = 0xFF;
  bool has_layer_target = false;
  uint8_t layer_target_layer = 0;
  uint8_t layer_target_target = 0;
  uint8_t layer_target_speed = LAYER_SPEED_FULL;
  bool layer_target_has_speed = false;
  bool has_layer_haptic = false;
  uint8_t layer_haptic_layer = 0;
  uint16_t layer_haptic_config = 0;
#if DEBUG_DRIVE
  bool has_debug_gain = false;
  uint8_t debug_gain_index = 0;
  int16_t debug_gain_value = 0;
  bool has_debug_drive = false;
  uint8_t debug_drive_flags = 0;
  uint8_t debug_drive_duty = 0;
#endif

  // Atomically copy all i2c requests in a single critical section
  noInterrupts();

  clear_error = i2c_clear_error_request;
  i2c_clear_error_request = false;

  self_cal = i2c_self_cal_request;
  i2c_self_cal_request = false;

  layer_change = i2c_layer_change_request;
  i2c_layer_change_request = 0xFF;

  if (i2c_layer_target_write.valid) {
    has_layer_target = true;
    layer_target_layer = i2c_layer_target_write.layer;
    layer_target_target = i2c_layer_target_write.target;
    layer_target_speed = i2c_layer_target_write.speed;
    layer_target_has_speed = i2c_layer_target_write.has_speed;
    i2c_layer_target_write.valid = false;
  }

#if DEBUG_DRIVE
  if (i2c_debug_gain_valid) {
    has_debug_gain = true;
    debug_gain_index = i2c_debug_gain_index;
    debug_gain_value = i2c_debug_gain_value;
    i2c_debug_gain_valid = false;
  }
  if (i2c_debug_drive_valid) {
    has_debug_drive = true;
    debug_drive_flags = i2c_debug_drive_flags;
    debug_drive_duty = i2c_debug_drive_duty;
    i2c_debug_drive_valid = false;
  }
#endif

  if (i2c_layer_haptic_write.valid) {
    has_layer_haptic = true;
    layer_haptic_layer = i2c_layer_haptic_write.layer;
    layer_haptic_config = i2c_layer_haptic_write.config;
    i2c_layer_haptic_write.valid = false;
  }

  interrupts();

  // Process all requests outside critical section
  if (clear_error) {
    if (get_mode() == MODE_ERROR) {
      set_mode(MODE_INPUT_IDLE);
    }
  }

  if (self_cal) {
    if (get_mode() != MODE_ERROR) {
      self_calibration_stage = 0;
      self_calibration_start = millis();
      set_mode(MODE_SELF_CALIBRATION);
    }
  }

  if (has_layer_target) {
    if (layer_target_has_speed) {
      layer_speeds[layer_target_layer & 0x07] = layer_target_speed;
    }
    write_layer_target(layer_target_layer, layer_target_target);
  }

  if (layer_change != 0xFF) {
    request_layer_change(layer_change);
  }

  if (has_layer_haptic) {
    write_layer_haptic_config(layer_haptic_layer, layer_haptic_config);
  }

#if DEBUG_DRIVE
  if (has_debug_gain) {
    switch (debug_gain_index) {
      case DEBUG_GAIN_VREF_SLOPE: move_vref_slope = debug_gain_value / 1000.0f; break;
      case DEBUG_GAIN_KV:         move_kv = debug_gain_value / 1000.0f; break;
      case DEBUG_GAIN_BD_RISING:  move_bd[MOTORCAL_RISING] = debug_gain_value; break;
      case DEBUG_GAIN_BD_FALLING: move_bd[MOTORCAL_FALLING] = debug_gain_value; break;
      case DEBUG_GAIN_K:
        move_inv_k[0] = move_inv_k[1] = 1.0f / (float)(debug_gain_value ? debug_gain_value : 1);
        break;
      case DEBUG_GAIN_VEL_MIN:    move_vel_min = debug_gain_value; break;
      case DEBUG_GAIN_DEADBAND:   move_deadband = debug_gain_value / 1000.0f; break;
      case DEBUG_GAIN_RAMP_RATE:  move_ramp_rate = debug_gain_value; break;
      case DEBUG_GAIN_TAKEUP:     move_takeup_duty = debug_gain_value; break;
      case DEBUG_GAIN_TAKEUP_RAMP: move_takeup_ramp_rate = debug_gain_value; break;
      // Override the calibration bounds (RAM only, not persisted) so a
      // miscalibrated fader - one whose stored range exceeds its physical
      // travel - can be reproduced on a good unit.
      case DEBUG_GAIN_CALIB_MIN:  input_calib_min = debug_gain_value; break;
      case DEBUG_GAIN_CALIB_MAX:  input_calib_max = debug_gain_value; break;
      case DEBUG_GAIN_TICK_US:
        if (debug_gain_value >= 100 && debug_gain_value <= 5000) {
          control_tick_period_us = debug_gain_value;
        }
        break;
    }
  }

  if (has_debug_drive) {
    if (debug_drive_flags == DEBUG_DRIVE_FLAGS_EXIT) {
      debug_drive_active = false;
      debug_drive_value = 0;
      motor_set(0, false, MOTOR_IDLE_COAST);
      // Hand both pins back to the timer, since the closed-loop paths drive
      // HCMPn directly and assume both compare outputs stay enabled.
      TCA0.SPLIT.CTRLB = (TCA_SPLIT_HCMP1EN_bm | TCA_SPLIT_HCMP2EN_bm);
      set_mode(MODE_INPUT_IDLE);
    } else {
      uint8_t dir = debug_drive_flags & DEBUG_DRIVE_DIR_bm;
      debug_drive_slow_decay = (debug_drive_flags & DEBUG_DRIVE_SLOW_DECAY_bm) != 0;
      debug_drive_value = (dir == DEBUG_DRIVE_DIR_A) ? (int16_t)debug_drive_duty
                        : (dir == DEBUG_DRIVE_DIR_B) ? -(int16_t)debug_drive_duty
                        : 0;
      debug_drive_brake = (dir == DEBUG_DRIVE_DIR_BRAKE);
      // PWM prescaler select, so fast/slow decay can be compared at each carrier
      uint8_t clk = (debug_drive_flags & DEBUG_DRIVE_CLK_bm) >> DEBUG_DRIVE_CLK_bp;
      uint8_t clksel = (clk == 0) ? TCA_SPLIT_CLKSEL_DIV4_gc
                     : (clk == 1) ? TCA_SPLIT_CLKSEL_DIV256_gc
                     : (clk == 2) ? TCA_SPLIT_CLKSEL_DIV2_gc
                                  : TCA_SPLIT_CLKSEL_DIV8_gc;
      TCA0.SPLIT.CTRLA = TCA_SPLIT_ENABLE_bm | clksel;
      debug_drive_active = true;
      debug_drive_last_update = millis();
    }
  }
#endif
}

void loop() {
  // Process any I2C requests that were queued by ISR callbacks
  process_i2c_requests();

  if (pending_calibrate_touch) {
    pending_calibrate_touch = false;
    motor_set(0, false, MOTOR_IDLE_COAST);
    delay(10);
    ptc_node_request_recal(&touch_sensor);
    // for (uint8_t i = 0; i < 4; i++) {
    //   digitalWrite(PIN_LED, HIGH);
    //   delay(100);
    //   digitalWrite(PIN_LED, LOW);
    //   delay(100);
    // }
  }
  // Keep the position filter fed from the free-running ADC on every pass, but
  // run the control law on a fixed tick so gains and filter constants have
  // real units instead of being per-loop-iteration.
  adc_drain();
#if DEBUG_DRIVE
  debug_loop_count++;
#endif

  uint32_t now_us = micros();
  if (now_us - last_control_tick_us >= control_tick_period_us) {
    // dt must be the interval since the previous *execution*, not since the
    // scheduled tick time: when the loop can't keep up, the schedule falls
    // behind real time and using it would skew the velocity estimate.
    control_dt_us = now_us - last_control_exec_us;
    last_control_exec_us = now_us;
    last_control_tick_us += control_tick_period_us;
    // If we fell far behind (e.g. a long blocking call), resynchronise rather
    // than running a burst of catch-up ticks.
    if (now_us - last_control_tick_us >= (uint32_t)control_tick_period_us * 4) {
      last_control_tick_us = now_us;
    }
    motor_update();

    // Copy state to i2c_outgoing_state atomically for ISR reads
    noInterrupts();
    i2c_outgoing_state = state;
    interrupts();
  }

  ptc_process(millis());

  // Straight to the port: digitalWrite() is ~180 bytes of pin lookup for a
  // heartbeat LED on a part with no flash to spare.
  if ((motor_drive_value != 0) || (millis() % 512 < 128)) {
    VPORTB.OUT |= LED_bm;
  } else {
    VPORTB.OUT &= ~LED_bm;
  }

}

// callback that is called by ptc_process at different points to ease user interaction
void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t* node) {
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    // MySerial.print("node touched:");
    // MySerial.println(ptc_get_node_id(node));

    // touch = true;
    touch_state_change_millis = millis();
    state |= STATE_TOUCH_bm;

    // Tap detection
    if (tap_state == TAP_NONE) {
      // First tap touch detected
      tap_timestamp = millis();
      tap_position_start = ADC1.RES;  // Store raw ADC value (no EWMA latency)
      tap_state = TAP_FIRST_PRESSED;
    } else if (tap_state == TAP_WAITING_FOR_DOUBLE) {
      uint32_t now = millis();
      if (now - tap_timestamp <= DOUBLE_TAP_MAX_INTERVAL &&
          tap_position_delta() <= TAP_MAX_MOVEMENT) {
        tap_timestamp = now;
        tap_state = TAP_SECOND_PRESSED;
      } else {
        // Too slow for double-tap, reset
        reset_tap_detection();
      }
    }
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    // MySerial.print("node released:");
    // MySerial.println(ptc_get_node_id(node));

    // touch = false;
    touch_state_change_millis = millis();
    state &= ~STATE_TOUCH_bm;

    // Tap detection: validate tap on release
    if (tap_state == TAP_FIRST_PRESSED || tap_state == TAP_SECOND_PRESSED) {
      uint32_t now = millis();
      uint32_t tap_duration = now - tap_timestamp;

      // Validate tap duration and movement
      if (tap_duration <= TAP_MAX_DURATION && tap_position_delta() <= TAP_MAX_MOVEMENT) {
        // Valid tap!
        if (tap_state == TAP_FIRST_PRESSED) {
          // First tap complete, wait for possible double-tap
          tap_timestamp = now;
          tap_state = TAP_WAITING_FOR_DOUBLE;
        } else if (tap_state == TAP_SECOND_PRESSED) {
          // Double-tap complete!

          // // Haptic kick for double-tap confirmation
          // TCA0.SPLIT.HCMP1 = 250;  // Motor direction A
          // TCA0.SPLIT.HCMP2 = 0;
          // delay(6);
          // TCA0.SPLIT.HCMP1 = 0;
          // TCA0.SPLIT.HCMP2 = 250;  // Motor direction B
          // delay(6);
          // TCA0.SPLIT.HCMP1 = 0;    // Stop
          // TCA0.SPLIT.HCMP2 = 0;

          increment_double_tap_nonce();
          reset_tap_detection();

          // Force state to idle after double-tap completion
          input_last_change_millis = now - IDLE_DURATION_THRESHOLD;
          set_mode(MODE_INPUT_IDLE);
        }
      } else {
        // Invalid tap (too long or too much movement)
        reset_tap_detection();
      }
    }
  } else if (PTC_CB_EVENT_CONV_SELF_CMPL == eventType) {
    // Do more complex things here
  } else if (PTC_CB_EVENT_CONV_CALIB & eventType) {
    // Increment recalibration counter on successful calibration
    if (eventType == PTC_CB_EVENT_CONV_CALIB) {
      touch_recal_count++;
    }
    // if (PTC_CB_EVENT_ERR_CALIB_LOW == eventType) {
    //   MySerial.print("Calib error, Cc too low.");
    // } else if (PTC_CB_EVENT_ERR_CALIB_HIGH == eventType) {
    //   MySerial.print("Calib error, Cc too high.");
    // } else if (PTC_CB_EVENT_ERR_CALIB_TO == eventType) {
    //   MySerial.print("Calib error, calculation timeout.");
    // } else {
    //   MySerial.print("Calib Successful.");
    // }
    // MySerial.print(" Node: ");
    // MySerial.println(ptc_get_node_id(node));
  }
}
