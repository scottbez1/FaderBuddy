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
#include <Wire.h>
#include <ptc_touch.h>
#include <megaTinyCore.h>
#include <EEPROM.h>

#include "shared/i2c_data.h"
#include "util.h"



#define DEMO 0

#define PIN_LED (PIN_PB2)

#define PIN_MOTOR_nSLEEP (PIN_PB3)

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

// Fixed-rate control tick. The control law needs a known, constant period so
// the derivative term and the filter time constants mean the same thing
// regardless of how long touch processing takes on a given loop iteration.
#define CONTROL_TICK_US (500)           // 2 kHz (leaves loop headroom for touch)
#define VELOCITY_TAU_S (0.004f)         // velocity estimate smoothing, seconds

// Remote-movement control gains, in ADC counts (see MODEL notes below).
// The plant is close to a velocity source with a friction deadband: above a
// breakaway duty, speed is roughly proportional to duty; below it nothing
// moves at all. Coast-down is first order with tau ~5 ms, so stopping distance
// is ~v * 6 ms - which is why arriving at full speed overshoots by ~40 ADC
// counts no matter how hard the bridge brakes afterwards.
#define MOVE_KP (2.0f)                  // duty per ADC count of error
#define MOVE_KD (0.04f)                 // duty per (ADC count/sec) of velocity
// KD also sets the tolerance to added carriage mass (e.g. a heavier knob cap):
// drive is cut at v = (KP/KD)*error, and overshoot appears once that ratio
// approaches 1/tau. At KP/KD = 50/s against a measured 1/tau of ~200/s there is
// roughly a 2-2.5x margin on effective moving mass before ringing returns.
// Friction feedforward, per direction. These are deliberately set ABOVE the
// breakaway duty measured on any one fader (68/88 on the reference unit), and
// are centred in the range that keeps every move settling rather than tuned for
// best accuracy on one fader. Rationale: the failure that matters is a fader
// that never settles and times out, and that happens when FF lands BELOW a
// unit's breakaway - the carriage then stalls just outside the deadband, waits
// for the stiction ramp, and escapes with a jump it cannot stop inside the
// window. Overshooting FF upward is cheap because MOVE_TAKEUP_DUTY caps drive
// while stalled, so an over-high FF is clamped rather than violent.
// Measured settling window on the reference fader is FF 66..146 rising; 106 is
// its centre, giving roughly +/-40 duty counts of breakaway tolerance instead
// of the +/-10 that a "just under breakaway" value gives. Accuracy is better at
// the centre too. Do not "optimise" these down against a single fader.
#define MOVE_FF_RISING (106)            // centred in the stable window
#define MOVE_FF_FALLING (124)           // same, plus the measured direction offset
// On-target window. This has a hard floor set by the plant, not by taste:
// nothing moves below breakaway duty, and at breakaway speed jumps straight to
// ~900 raw/s, so the smallest correction the fader can make is roughly that
// speed times the reaction+stopping time - about 10-16 raw, i.e. 5-8 ADC
// counts. A deadband below that floor cannot be satisfied on a fader whose
// breakaway differs from the feedforward constants: the controller steps past
// the window in both directions forever and eventually trips the movement
// timeout. Keep this at or above the floor; it is ~1.5 LSB of the 8-bit
// position the host sees, so tightening it buys nothing a host can observe.
#define MOVE_DEADBAND (6.0f)            // ADC counts considered "on target"
#define MOVE_MAX_DUTY (254)

// Optional speed limiting. Cruise velocity is already proportional to error
// (the plant is a velocity source, so v ~ kv*KP*e / (1 + kv*KD)), which is why
// big moves are fast and small ones are slow. Capping the error fed to the P
// term therefore caps cruise speed, with no extra state and no motion profile.
// Near the target the error is below the cap, so the clamp is inactive and the
// tuned decel/coast/settle behaviour is untouched.
//
// This is a limit, not a precise speed: the constant below is the measured
// closed-loop velocity per unit of error on the reference fader, and it varies
// by ~20% between directions (friction is direction-dependent) plus unit to
// unit. Callers wanting an exact velocity need a governor closing the loop on
// measured velocity instead.
// Implemented as a one-sided governor on measured velocity rather than by
// clamping the error: the feedforward sits well above breakaway (deliberately,
// for hardware tolerance), so it alone commands ~2200 raw counts/sec. Clamping
// the error can only slow the fader to that floor, whereas subtracting from the
// drive pulls it below the feedforward and reaches the real limit - the
// Stribeck floor of ~900 raw counts/sec, below which the mechanism stick-slips
// rather than moving smoothly.
// Two cooperating parts are needed, and neither works alone:
//  - the error clamp stops the P term saturating. For a large move KP*error is
//    ~1300 duty, so the governor below cannot pull it under the 254 ceiling.
//  - the governor pulls drive below the feedforward. The feedforward alone
//    commands ~2200 raw counts/sec, so clamping the error can only slow the
//    fader to that floor, never under it.
#define MOVE_VELOCITY_PER_ERROR (30.0f)  // ADC counts/sec per ADC count of error
#define MOVE_VEL_LIMIT_GAIN (0.08f)      // duty per (ADC count/sec) of overspeed
// Slowest speed the mechanism sustains smoothly. Below roughly this the motor
// cannot maintain continuous rotation (Stribeck friction) and creeps in
// stick-slip steps instead, so requests slower than this are clamped rather
// than honoured. Measured floor is ~150 position counts/sec, i.e. full travel
// in about 1.7 s.
#define MOVE_VEL_MIN (560.0f)            // ADC counts/sec
#define MOVE_VEL_UNLIMITED (0.0f)

// On movement timeout, an error this small means the fader is essentially in
// position and merely hunting - give up quietly rather than latching
// MODE_ERROR, which needs a host round-trip to clear. Anything larger is a
// genuine fault and still errors.
#define MOVE_TIMEOUT_TOLERANCE (20.0f)  // ADC counts

// Stiction ramp. The feedforward alone leaves a dead region: for small errors
// FF + KP*error can sit below this unit's actual breakaway duty, so the
// controller commands motion it cannot produce and the move never completes
// (previously this timed out into MODE_ERROR). Rather than raise the
// feedforward - which overshoots on a looser fader, since anything above
// breakaway jumps straight to ~900 raw counts/sec - ramp in extra drive only
// while we are asking for motion and not getting it, and drop it the moment
// the carriage breaks free. This finds whatever breakaway a given fader has
// instead of relying on a per-unit constant.
#define MOVE_STALL_VELOCITY (60.0f)     // ADC counts/sec below which we're stalled
#define MOVE_RAMP_RATE (250.0f)         // duty per second of ramp-in
#define MOVE_RAMP_MAX (70.0f)           // ceiling, so a jam can't wind up to full drive

// Backlash take-up. There is a little slack in the belt, so a move that
// reverses direction starts with the motor unloaded. Commanding full duty into
// that slack lets the rotor spin up freely and then snap taut, which is both an
// audible click and a jerk at the carriage. While we are still stalled the
// drive is therefore capped just above breakaway: enough to cross the slack
// promptly, but slow enough that engagement is gentle. Full authority returns
// as soon as the carriage is actually moving, so this costs only a few ms.
// Backlash take-up. A direction-reversed move starts with the motor unloaded,
// so commanding full duty lets the rotor spin up through the slack and snap it
// taut - an audible click and a jerk at the carriage. The drive ceiling
// therefore starts low at the beginning of every move and opens up over time.
//
// This is deliberately time-based rather than gated on "are we moving yet":
// while crossing the slack the rotor IS moving (it just isn't loaded), so a
// velocity-gated ceiling lifts partway through the slack and full duty still
// lands on engagement. Time is the only signal available that does not depend
// on whether the belt has taken up yet.
// MUST stay above MOVE_FF_RISING/FALLING. The ceiling exists to stop the drive
// stepping to full duty, not to suppress the feedforward: set below FF it
// starves the very term that gets the carriage moving, so the fader stalls,
// waits for the stiction ramp, and breaks free abruptly - which both hunts and
// makes the click worse rather than better.
#define MOVE_TAKEUP_DUTY (130)           // duty ceiling at the start of a move
#define MOVE_TAKEUP_RAMP_RATE (1200.0f)  // duty per second the ceiling opens up

#if DEBUG_DRIVE
// Runtime-tunable copies, so gain sweeps don't need a reflash per trial.
float move_kp = MOVE_KP;
float move_kd = MOVE_KD;
int16_t move_ff_rising = MOVE_FF_RISING;
int16_t move_ff_falling = MOVE_FF_FALLING;
float move_deadband = MOVE_DEADBAND;
float move_ramp_rate = MOVE_RAMP_RATE;
int16_t move_takeup_duty = MOVE_TAKEUP_DUTY;
float move_takeup_ramp_rate = MOVE_TAKEUP_RAMP_RATE;
#else
#define move_kp MOVE_KP
#define move_kd MOVE_KD
#define move_ff_rising MOVE_FF_RISING
#define move_ff_falling MOVE_FF_FALLING
#define move_deadband MOVE_DEADBAND
#define move_ramp_rate MOVE_RAMP_RATE
#define move_takeup_duty MOVE_TAKEUP_DUTY
#define move_takeup_ramp_rate MOVE_TAKEUP_RAMP_RATE
#endif

const float ALPHA = 0.05;
float input_ewma = 0;
float stiction_ramp = 0;                // extra drive ramped in while stalled
float move_max_velocity = MOVE_VEL_UNLIMITED;  // 0 = unlimited
float move_max_error = 1023.0f;         // matching P-term clamp; large = inactive
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

// EEPROM calibration storage
#define EEPROM_CALIBRATION_ADDR 0
#define EEPROM_CALIBRATION_MAGIC 0xCAFE  // Magic number to validate EEPROM data

struct CalibrationData {
  uint16_t magic;         // Magic number for validation
  uint16_t calib_min;     // Minimum ADC value (fader at one end)
  uint16_t calib_max;     // Maximum ADC value (fader at other end)
  uint16_t checksum;      // Simple checksum for data integrity
};

uint16_t input_calib_min = 40;
uint16_t input_calib_max = 1010;

int16_t target_adc = 512;
uint8_t current_register = REG_VERSION;  // Track which register was last accessed

// Haptic configuration storage
uint32_t haptic_config = 0;  // Bit-packed haptic configuration register (currently active layer's config)
uint8_t last_haptic_nonce = 0;  // Track last seen nonce to detect changes [DEPRECATED in v5]

// Layer state storage (27 bytes total) - Protocol v5+
uint16_t layer_haptic_configs[8];      // 16 bytes - 16-bit haptic config per layer
uint8_t layer_restore_positions[8];    // 8 bytes - restore position per layer (0-255)
uint8_t layer_move_times[8];           // 8 bytes - optional full-travel move time per layer (0 = unlimited)
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
  uint8_t move_time; // optional full-travel move time (see LAYER_MOVE_TIME)
  bool has_move_time;// false = 3-byte legacy write, leave the limit as-is
  bool valid;
};
volatile LayerTargetWrite i2c_layer_target_write = {0, 0, 0, false, false};

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

bool pending_report_on_idle = false;

bool pending_calibrate_touch = false;
void calibrate_touch();
void reset_tap_detection();

// Forward declarations for layer management functions (Protocol v5+)
void request_layer_change(uint8_t new_layer);
void apply_layer_change(uint8_t new_layer);
void write_layer_target(uint8_t layer, uint8_t target);
void apply_move_time(uint8_t move_time_10ms);
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

// I2C request handler - called when master requests data
// IMPORTANT: This runs in ISR context - only read i2c_ prefixed state!
void onI2cRequest() {
  uint8_t r = current_register;
  if (r == REG_VERSION) {
      Wire.write(I2C_PROTOCOL_VERSION);
  } else if (r == REG_STATE) {
      // Snapshot once: the main loop can update i2c_outgoing_state between
      // byte writes, which would hand the controller a torn value.
      uint32_t s = i2c_outgoing_state;
      Wire.write((s >> 24) & 0xFF);
      Wire.write((s >> 16) & 0xFF);
      Wire.write((s >> 8) & 0xFF);
      Wire.write(s & 0xFF);
  } else if (r == REG_UPTIME) {
      uint32_t uptime = millis();
      Wire.write((uptime >> 24) & 0xFF);
      Wire.write((uptime >> 16) & 0xFF);
      Wire.write((uptime >> 8) & 0xFF);
      Wire.write(uptime & 0xFF);
  } else if (r == REG_TOUCH_RAW) {
      uint16_t touch_raw = touch_sensor.sensorData;
      Wire.write((touch_raw >> 8) & 0xFF);  // High byte
      Wire.write(touch_raw & 0xFF);         // Low byte
  } else if (r == REG_SERIAL) {
      // Read 10-byte serial number from SIGROW
      Wire.write(SIGROW.SERNUM0);
      Wire.write(SIGROW.SERNUM1);
      Wire.write(SIGROW.SERNUM2);
      Wire.write(SIGROW.SERNUM3);
      Wire.write(SIGROW.SERNUM4);
      Wire.write(SIGROW.SERNUM5);
      Wire.write(SIGROW.SERNUM6);
      Wire.write(SIGROW.SERNUM7);
      Wire.write(SIGROW.SERNUM8);
      Wire.write(SIGROW.SERNUM9);
  } else if (r == REG_TOUCH_DELTA) {
      // Touch delta (signed 16-bit): sensorData - reference
      int16_t delta = ptc_get_node_delta(&touch_sensor);
      Wire.write((delta >> 8) & 0xFF);  // High byte
      Wire.write(delta & 0xFF);         // Low byte
  } else if (r == REG_TOUCH_REF) {
      // Touch reference value (unsigned 16-bit)
      uint16_t reference = touch_sensor.reference;
      Wire.write((reference >> 8) & 0xFF);  // High byte
      Wire.write(reference & 0xFF);         // Low byte
  } else if (r == REG_TOUCH_RECAL) {
      // Touch recalibration count (unsigned 16-bit)
      Wire.write((touch_recal_count >> 8) & 0xFF);  // High byte
      Wire.write(touch_recal_count & 0xFF);         // Low byte
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
      for (uint8_t i = 0; i < 9; i++) {
        Wire.write((vals[i] >> 8) & 0xFF);
        Wire.write(vals[i] & 0xFF);
      }
#endif
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
        // Write: register + layer + target position [+ optional speed limit].
        // The 3-byte form is the original protocol and leaves the layer's
        // existing speed limit alone, so old controllers are unaffected.
        i2c_layer_target_write.layer = Wire.read() & 0x07;
        i2c_layer_target_write.target = Wire.read();
        i2c_layer_target_write.has_move_time = (howMany == 4);
        i2c_layer_target_write.move_time = (howMany == 4) ? Wire.read() : 0;
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

// Save calibration data to EEPROM
void saveCalibration() {
  CalibrationData cal;
  cal.magic = EEPROM_CALIBRATION_MAGIC;
  cal.calib_min = input_calib_min;
  cal.calib_max = input_calib_max;
  // Simple checksum: sum of all data
  cal.checksum = cal.magic + cal.calib_min + cal.calib_max;

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
  uint16_t expected_checksum = cal.magic + cal.calib_min + cal.calib_max;
  if (cal.checksum != expected_checksum) {
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

  return true;
}

void setup_i2c() {
  pinMode(PIN_ADDR_0, INPUT_PULLUP);
  pinMode(PIN_ADDR_1, INPUT_PULLUP);
  pinMode(PIN_ADDR_2, INPUT_PULLUP);

  uint8_t address = I2C_BASE_ADDRESS +
    (!digitalRead(PIN_ADDR_2) << 2) +
    (!digitalRead(PIN_ADDR_1) << 1) +
    (!digitalRead(PIN_ADDR_0) << 0);

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

// Apply layer change and start movement to new layer's restore position
void apply_layer_change(uint8_t new_layer) {
  // Start movement to new layer's restore position
  target_adc = BOUNDED_LERP_UINT16(
    layer_restore_positions[new_layer], 0, 255,
    input_calib_min, input_calib_max
  );
  apply_move_time(layer_move_times[new_layer]);
  remote_movement_start = millis();
  remote_movement_start_position = input_ewma;
  remote_movement_steady_start = millis();
  set_mode(MODE_REMOTE_MOVEMENT_IN_PROGRESS);

  // Load new layer's haptic config
  haptic_config = layer_haptic_configs[new_layer];

  pending_layer_change = 0xFF;
  active_layer = new_layer;
}

// Write target position to a specific layer
void write_layer_target(uint8_t layer, uint8_t target) {
  if (layer > 7) return;

  if (layer == active_layer) {
    Mode mode = get_mode();
    if (mode == MODE_INPUT_IDLE) {
      // Start remote movement
      layer_restore_positions[layer] = target;
      target_adc = BOUNDED_LERP_UINT16(target, 0, 255, input_calib_min, input_calib_max);
      apply_move_time(layer_move_times[layer]);
      set_mode(MODE_REMOTE_MOVEMENT_IN_PROGRESS);
      remote_movement_start = millis();
      remote_movement_start_position = input_ewma;
      remote_movement_steady_start = millis();
    } else if (mode == MODE_INPUT_ACTIVE) {
      // Ignore - user has control
    } else if (mode == MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
      // Update target of in-progress movement
      layer_restore_positions[layer] = target;
      target_adc = BOUNDED_LERP_UINT16(target, 0, 255, input_calib_min, input_calib_max);
      apply_move_time(layer_move_times[layer]);
      remote_movement_start = millis();
      remote_movement_start_position = input_ewma;
      remote_movement_steady_start = millis();
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

// Translate a layer's move-time byte into the velocity limit used by the
// control law. The byte is the time a full-scale (0-255) move should take, in
// units of 10 ms; 0 means unlimited. Since the whole calibrated span is crossed
// in that time, the velocity is simply span/time.
void apply_move_time(uint8_t move_time_10ms) {
  if (move_time_10ms == LAYER_MOVE_TIME_UNLIMITED) {
    move_max_velocity = MOVE_VEL_UNLIMITED;
    move_max_error = 1023.0f;
    return;
  }
  uint16_t span = input_calib_max - input_calib_min;
  float ms = (float)move_time_10ms * LAYER_MOVE_TIME_MS_PER_UNIT;
  float adc_per_sec = (float)span * 1000.0f / ms;
  // Clamp to what the mechanism can actually sustain smoothly
  if (adc_per_sec < MOVE_VEL_MIN) adc_per_sec = MOVE_VEL_MIN;
  move_max_velocity = adc_per_sec;
  // Matching error clamp, so the proportional term cannot saturate past the
  // point where the governor has any authority.
  float clamp = adc_per_sec / MOVE_VELOCITY_PER_ERROR;
  if (clamp < MOVE_DEADBAND * 2) clamp = MOVE_DEADBAND * 2;
  move_max_error = clamp;
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
          // PD on position plus a friction feedforward, which covers the
          // breakaway duty so the controller only has to supply the part of
          // the drive that sets speed rather than fighting stiction. The D term
          // is what prevents overshoot: it cancels the feedforward while the
          // carriage is still moving fast, cutting drive roughly one stopping
          // distance short of the target so it coasts in.

          // Clamp the error driving the P term when a speed limit is active.
          // Inactive near the target and when unlimited, so the tuned approach
          // and settle behaviour is unchanged.
          float e_ctrl = error;
          if (e_ctrl > move_max_error) e_ctrl = move_max_error;
          if (e_ctrl < -move_max_error) e_ctrl = -move_max_error;

          float u = move_kp * e_ctrl - move_kd * velocity_ewma;
          u += (error > 0) ? move_ff_rising : -move_ff_falling;

          // Ramp in extra drive only while stalled, and shed it as soon as the
          // carriage moves, so the ramp never contributes to overshoot.
          float speed = velocity_ewma < 0 ? -velocity_ewma : velocity_ewma;
          if (speed < MOVE_STALL_VELOCITY) {
            stiction_ramp += move_ramp_rate * (control_dt_us * 1e-6f);
            if (stiction_ramp > MOVE_RAMP_MAX) stiction_ramp = MOVE_RAMP_MAX;
          } else {
            stiction_ramp = 0;
          }
          u += (error > 0) ? stiction_ramp : -stiction_ramp;

          // Optional velocity limit: bleed off drive only while over the
          // requested speed, so approach and settle near the target (already
          // slower than any useful limit) are unaffected.
          if (move_max_velocity > MOVE_VEL_UNLIMITED) {
            float speed_now = velocity_ewma < 0 ? -velocity_ewma : velocity_ewma;
            if (speed_now > move_max_velocity) {
              float excess = MOVE_VEL_LIMIT_GAIN * (speed_now - move_max_velocity);
              u += (velocity_ewma > 0) ? -excess : excess;
            }
          }

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
    case MODE_SELF_CALIBRATION:
      switch (self_calibration_stage) {
        case 0:
          if (now > self_calibration_start + SELF_CALIBRATION_TIMEOUT) {
            self_calibration_adc_stage_0 = adc_val;
            self_calibration_stage++;
            self_calibration_start = millis();
          } else {
            // Move toward lower ADC value
            motor_set(-254, true, MOTOR_IDLE_COAST);
          }
          break;
        case 1:
          if (now > self_calibration_start + SELF_CALIBRATION_TIMEOUT) {
            self_calibration_adc_stage_1 = adc_val;
            self_calibration_stage++;
            self_calibration_start = millis();
          } else {
            // Move toward higher ADC value
            motor_set(254, true, MOTOR_IDLE_COAST);
          }
          break;
        case 2:
          motor_set(0, false, MOTOR_IDLE_COAST);
          if (abs((int16_t)self_calibration_adc_stage_0 - self_calibration_adc_stage_1) < 900) {
            set_mode(Mode::MODE_ERROR);
          } else {
            // Apply calibration in memory
            // Divide by 2 to account for 2-sample ADC aggregation (adc_val is 2x, but input_ewma is corrected)
            input_calib_min = (SELF_CALIBRATION_BUFFER * self_calibration_adc_stage_0 + (1.0 - SELF_CALIBRATION_BUFFER) * self_calibration_adc_stage_1) / 2;
            input_calib_max = (SELF_CALIBRATION_BUFFER * self_calibration_adc_stage_1 + (1.0 - SELF_CALIBRATION_BUFFER) * self_calibration_adc_stage_0) / 2;

            // Save calibration to EEPROM
            saveCalibration();

            // Since we moved the position, do a remote movement to the previous target
            remote_movement_start = millis();
            remote_movement_start_position = input_ewma;
            remote_movement_steady_start = millis();
            // TODO: re-calculate target_adc using the new calibration bounds. Can't do that now since we lerp target to an ADC value upon receipt, without saving the 0-255 value
            set_mode(Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS);
          }
          break;
      }
      break;
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

  // Load calibration from EEPROM if available
  if (loadCalibration()) {
#if SERIAL_ENABLED
    Serial.print("Loaded calibration from EEPROM: min=");
    Serial.print(input_calib_min);
    Serial.print(", max=");
    Serial.println(input_calib_max);
#endif
  } else {
#if SERIAL_ENABLED
    Serial.println("No valid calibration found, using defaults");
#endif
  }

  // Initialize all layers with default configuration (Protocol v5+)
  for (uint8_t i = 0; i < 8; i++) {
    layer_haptic_configs[i] = 0;  // Default: HAPTIC_NO_HAPTICS (smooth mode), all bits 0
    layer_restore_positions[i] = 128;  // Default: midpoint
    layer_move_times[i] = LAYER_MOVE_TIME_UNLIMITED;
  }
  active_layer = 0;
  pending_layer_change = 0xFF;  // No pending change
  queried_layer = 0;

  // Load active layer's haptic config into global haptic_config
  haptic_config = layer_haptic_configs[0];

  setup_i2c();

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_FADER, INPUT);
  pinMode(PIN_MOTOR_nSLEEP, OUTPUT);
  pinMode(PIN_MOTOR_A, OUTPUT);
  pinMode(PIN_MOTOR_B, OUTPUT);

  // Set up TCA0 for high-frequency PWM
  setup_tca0();
  
  digitalWrite(PIN_MOTOR_nSLEEP, HIGH);
  digitalWrite(PIN_LED, HIGH);

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
  uint8_t layer_target_move_time = 0;
  bool layer_target_has_move_time = false;
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
    layer_target_move_time = i2c_layer_target_write.move_time;
    layer_target_has_move_time = i2c_layer_target_write.has_move_time;
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
    if (layer_target_has_move_time) {
      layer_move_times[layer_target_layer & 0x07] = layer_target_move_time;
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
      case DEBUG_GAIN_KP:         move_kp = debug_gain_value / 1000.0f; break;
      case DEBUG_GAIN_KD:         move_kd = debug_gain_value / 1000.0f; break;
      case DEBUG_GAIN_FF_RISING:  move_ff_rising = debug_gain_value; break;
      case DEBUG_GAIN_FF_FALLING: move_ff_falling = debug_gain_value; break;
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

  // digitalWrite(PIN_LED, (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp);
  digitalWrite(PIN_LED, (motor_drive_value != 0) || (millis() % 512 < 128));
  // digitalWrite(PIN_LED, millis()%512 < 128 || touch);

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
      // Use raw ADC for immediate movement detection (no EWMA latency)
      uint16_t current_adc = ADC1.RES;
      uint16_t position_change = (current_adc > tap_position_start) ?
                                  (current_adc - tap_position_start) :
                                  (tap_position_start - current_adc);

      uint32_t now = millis();
      if (now - tap_timestamp <= DOUBLE_TAP_MAX_INTERVAL && position_change <= TAP_MAX_MOVEMENT) {
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

      // Use raw ADC for immediate movement detection (no EWMA latency)
      uint16_t current_adc = ADC1.RES;
      uint16_t position_change = (current_adc > tap_position_start) ?
                                  (current_adc - tap_position_start) :
                                  (tap_position_start - current_adc);

      // Validate tap duration and movement
      if (tap_duration <= TAP_MAX_DURATION && position_change <= TAP_MAX_MOVEMENT) {
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
