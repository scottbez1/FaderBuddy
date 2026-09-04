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

#pragma once

#include <stdint.h>

#define I2C_PROTOCOL_VERSION (5)  // v5: Layer management in firmware, 16-bit haptic config


/*
 * I2C Register Map (Protocol Version 5)
 * ======================================
 *
 * Addr | Register Name       | Access  | Type | Description
 * -----|---------------------|---------|------|------------
 * 0x00 | VERSION             | R       | u8   | Protocol version (currently 5)
 * -----|---------------------|---------|------|------------
 * 0x01 | STATE               | R       | u32  | Current state (includes active layer in bits 4-6)
 * -----|---------------------|---------|------|------------
 * 0x02 | (removed in v5)     |         |      | Use REG_LAYER_TARGET instead
 * -----|---------------------|---------|------|------------
 * 0x03 | UPTIME              | R       | u32  | Uptime milliseconds
 * -----|---------------------|---------|------|------------
 * 0x04 | CAL_TOUCH           | W       | N/A  | Recalibrate touch sensor
 * -----|---------------------|---------|------|------------
 * 0x05 | CLEAR_ERROR         | W       | N/A  | Clear error state
 * -----|---------------------|---------|------|------------
 * 0x06 | TOUCH_RAW           | R       | u16  | Raw touch value
 * -----|---------------------|---------|------|------------
 * 0x07 | SELF_CAL            | W       | N/A  | Initiate self calibration of motor/potentiometer
 * -----|---------------------|---------|------|------------
 * 0x08 | SERIAL              | R       | u8[10] | Chip serial number (10 bytes)
 * -----|---------------------|---------|------|------------
 * 0x09 | TOUCH_DELTA         | R       | i16  | Touch delta (sensorData - reference)
 * -----|---------------------|---------|------|------------
 * 0x0A | TOUCH_REF           | R       | u16  | Touch reference value
 * -----|---------------------|---------|------|------------
 * 0x0B | TOUCH_RECAL         | R       | u16  | Touch recalibration count
 * -----|---------------------|---------|------|------------
 * 0x0C | HAPTIC_CONFIG       | R/W     | u32  | Haptic configuration [DEPRECATED - use REG_LAYER_HAPTIC_CONFIG]
 * -----|---------------------|---------|------|------------
 * 0x0D | ACTIVE_LAYER        | R/W     | u8   | Active layer index (0-7)
 * -----|---------------------|---------|------|------------
 * 0x0E | LAYER_TARGET        | R/W     | -    | Layer restore position + optional move time (see below)
 * -----|---------------------|---------|------|------------
 * 0x0F | LAYER_HAPTIC_CONFIG | R/W     | -    | Layer haptic config (layer-addressed, u16)
 * -----|---------------------|---------|------|------------
 * 0x10 | DEBUG_DRIVE         | W       | u8[2]| Open-loop motor drive (DEBUG_DRIVE builds only)
 * -----|---------------------|---------|------|------------
 * 0x11 | DEBUG_STATUS        | R       |u8[16]| Control-loop internals (DEBUG_DRIVE builds only)
 * -----|---------------------|---------|------|------------
 * 0x12 | DEBUG_GAINS         | W       | u8[3]| Runtime gain override (DEBUG_DRIVE builds only)
 * -----|---------------------|---------|------|------------
 *
 * Protocol:
 * - Simple registers:
 *   - Read:  Write register address, then read N bytes
 *   - Write: Write register address + N data bytes
 * - Layer-addressed registers (0x0E, 0x0F):
 *   - Read:  Write [register, layer], then read N bytes for that layer
 *   - Write: Write [register, layer, ...data] to write to specific layer
 *
 * LAYER_TARGET (0x0E) accepts an OPTIONAL trailing speed-limit byte:
 *   [0x0E, layer, position]           - move at full speed (unchanged behaviour)
 *   [0x0E, layer, position, move_time] - move no faster than this
 * Three-byte writes behave exactly as before, so this is backwards compatible
 * and does not change the protocol version. See LAYER_MOVE_TIME below.
 * - All multi-byte values are big-endian (MSB first)
 *
 * v5 Breaking Changes from v4:
 * - HAPTIC_CONFIG changed from 32-bit to 16-bit (removed nonce and target position)
 * - STATE register bits 4-6 changed from haptic_config_nonce to active_layer
 * - New layer-addressed protocol for per-layer configuration
 */

// I2C register addresses
#define REG_VERSION 0x00  // Protocol version
#define REG_STATE   0x01  // Current
// 0x02 removed in v5 - use REG_LAYER_TARGET instead
#define REG_UPTIME  0x03
#define REG_CAL_TOUCH 0x04
#define REG_CLEAR_ERROR 0x05
#define REG_TOUCH_RAW 0x06
#define REG_SELF_CAL 0x07
#define REG_SERIAL 0x08  // Chip serial number (10 bytes)
#define REG_TOUCH_DELTA 0x09  // Touch delta (signed 16-bit)
#define REG_TOUCH_REF 0x0A  // Touch reference value (unsigned 16-bit)
#define REG_TOUCH_RECAL 0x0B  // Touch recalibration count (unsigned 16-bit)
// 0x0C removed in v5 - use REG_LAYER_HAPTIC_CONFIG instead
#define REG_ACTIVE_LAYER 0x0D  // Active layer index (R/W, u8)
#define REG_LAYER_TARGET 0x0E  // Layer restore position (layer-addressed, R/W, u8)
#define REG_LAYER_HAPTIC_CONFIG 0x0F  // Layer haptic config (layer-addressed, R/W, u16)
/*
 * Optional move-time limit for LAYER_TARGET writes.
 *
 * The byte is the time a FULL-SCALE move (0 -> 255) should take, in units of
 * 10 ms. 0 means unlimited (the default, and what a 3-byte write leaves in
 * place). So 100 = "one second for full travel"; range 10 ms .. 2550 ms.
 *
 * It is a velocity cap, not a move scheduler: a shorter move takes
 * proportionally less time rather than being stretched to fill the duration.
 *
 * This is a LIMIT and not a precise speed. It is realised through a
 * friction-dependent plant, so actual timing varies with the fader - expect
 * within roughly 15% for full-travel times up to ~800 ms, and about 7%
 * difference between the two directions of travel.
 *
 * The mechanism cannot move smoothly below roughly 200 position counts/sec, so
 * requests slower than about 1.2 s of full travel are clamped rather than
 * honoured; below that speed the motor creeps in stick-slip steps instead of
 * moving continuously.
 */
#define LAYER_MOVE_TIME_UNLIMITED (0)
#define LAYER_MOVE_TIME_MS_PER_UNIT (10)

#define REG_DEBUG_DRIVE 0x10  // Open-loop motor drive (W, [flags, duty]); DEBUG_DRIVE builds only

/*
 * REG_DEBUG_DRIVE (0x10) - write-only, present only in DEBUG_DRIVE builds.
 *
 * Write [0x10, flags, duty] to command the H-bridge directly, bypassing the
 * control loop, for system identification. Writing flags = 0xFF exits open-loop
 * mode. The firmware coasts the motor if a command is not refreshed within
 * 400 ms, or if the fader nears a travel limit.
 */
#define DEBUG_DRIVE_FLAGS_EXIT (0xFF)

// Direction: 2 bits at position 0
#define DEBUG_DRIVE_DIR_bp (0)
#define DEBUG_DRIVE_DIR_bm (0x03 << DEBUG_DRIVE_DIR_bp)
#define DEBUG_DRIVE_DIR_COAST (0)
#define DEBUG_DRIVE_DIR_A     (1)  // Toward the motor end (rising ADC)
#define DEBUG_DRIVE_DIR_B     (2)  // Toward the low end (falling ADC)
#define DEBUG_DRIVE_DIR_BRAKE (3)  // Both low sides on (dynamic braking)

// Decay mode: 1 bit at position 2 (0 = fast/coast, 1 = slow/brake)
#define DEBUG_DRIVE_SLOW_DECAY_bp (2)
#define DEBUG_DRIVE_SLOW_DECAY_bm (1 << DEBUG_DRIVE_SLOW_DECAY_bp)

// PWM prescaler select: 2 bits at position 4
// 0 = DIV4 (19.6 kHz), 1 = DIV256 (306 Hz), 2 = DIV2 (39.2 kHz), 3 = DIV8 (9.8 kHz)
#define DEBUG_DRIVE_CLK_bp (4)
#define DEBUG_DRIVE_CLK_bm (0x03 << DEBUG_DRIVE_CLK_bp)

/*
 * REG_DEBUG_STATUS (0x11) - read-only, DEBUG_DRIVE builds only.
 * 16 bytes, big-endian: calib_min u16, calib_max u16, target_adc i16,
 * drive i16, velocity i16 (ADC counts/sec), error_x8 i16 (error * 8),
 * loop_hz u16, tick_hz u16.
 */
#define REG_DEBUG_STATUS 0x11

/*
 * REG_DEBUG_GAINS (0x12) - write-only, DEBUG_DRIVE builds only.
 * Write [0x12, index, value_hi, value_lo] to override one control-loop gain at
 * runtime, so tuning doesn't need a reflash per trial. Values are fixed point:
 * KP and KD are scaled by 1000, the rest are integers.
 */
#define REG_DEBUG_GAINS 0x12
#define DEBUG_GAIN_KP          (0)  // duty per ADC count, x1000
#define DEBUG_GAIN_KD          (1)  // duty per (ADC count/sec), x1000
#define DEBUG_GAIN_FF_RISING   (2)  // duty
#define DEBUG_GAIN_FF_FALLING  (3)  // duty
#define DEBUG_GAIN_DEADBAND    (4)  // ADC counts, x1000
#define DEBUG_GAIN_TICK_US     (5)  // control tick period, microseconds
#define DEBUG_GAIN_RAMP_RATE   (6)  // stiction ramp rate, duty per second
#define DEBUG_GAIN_TAKEUP      (7)  // backlash take-up duty ceiling (0 = disabled)
#define DEBUG_GAIN_TAKEUP_RAMP (10) // take-up ceiling ramp rate, duty per second
#define DEBUG_GAIN_CALIB_MIN   (8)  // override calib_min (RAM only, not saved)
#define DEBUG_GAIN_CALIB_MAX   (9)  // override calib_max (RAM only, not saved)

enum Mode : uint8_t {
  MODE_REMOTE_MOVEMENT_IN_PROGRESS = 0,
  MODE_INPUT_ACTIVE                = 1,
  MODE_INPUT_IDLE                  = 2,
  MODE_ERROR                       = 3,
  MODE_SELF_CALIBRATION            = 4,
};

enum HapticMode : uint8_t {
  HAPTIC_NO_HAPTICS              = 0,
  HAPTIC_SMOOTH_WITH_MAGNET_ENDS = 1,
  HAPTIC_DETENTS                 = 2,
};


/*
 * Bitfield definitions: bit position (_bp), bit size (_bs), bit mask (_bm)
 */

// Touch: 1 bit at position 0
#define STATE_TOUCH_bp (0)
#define STATE_TOUCH_bs (1)
#define STATE_TOUCH_bm (((1UL << STATE_TOUCH_bs) - 1) << STATE_TOUCH_bp)

// Mode: 3 bits at position 1
#define STATE_MODE_bp (1)
#define STATE_MODE_bs (3)
#define STATE_MODE_bm (((1UL << STATE_MODE_bs) - 1) << STATE_MODE_bp)

// Active layer: 3 bits at position 4 (replaces haptic_config_nonce in v5)
#define STATE_ACTIVE_LAYER_bp      (4)
#define STATE_ACTIVE_LAYER_bs      (3)
#define STATE_ACTIVE_LAYER_bm      (((1UL << STATE_ACTIVE_LAYER_bs) - 1) << STATE_ACTIVE_LAYER_bp)

// Position: 8 bits at position 7
#define STATE_POSITION_bp            (7)
#define STATE_POSITION_bs            (8)
#define STATE_POSITION_bm            (((1UL << STATE_POSITION_bs) - 1) << STATE_POSITION_bp)

// Position nonce: 2 bits at position 15
#define STATE_POSITION_NONCE_bp            (15)
#define STATE_POSITION_NONCE_bs            (2)
#define STATE_POSITION_NONCE_bm            (((1UL << STATE_POSITION_NONCE_bs) - 1) << STATE_POSITION_NONCE_bp)

// Raw ADC: 11 bits at position 17
#define STATE_RAW_ADC_bp            (17)
#define STATE_RAW_ADC_bs            (11)
#define STATE_RAW_ADC_bm            (((1UL << STATE_RAW_ADC_bs) - 1) << STATE_RAW_ADC_bp)

// Double tap nonce: 2 bits at position 28
#define STATE_DOUBLE_TAP_NONCE_bp   (28)
#define STATE_DOUBLE_TAP_NONCE_bs   (2)
#define STATE_DOUBLE_TAP_NONCE_bm   (((1UL << STATE_DOUBLE_TAP_NONCE_bs) - 1) << STATE_DOUBLE_TAP_NONCE_bp)

/*
 * Haptic Configuration Bitfields (16-bit format - PROTOCOL VERSION 5+)
 * =====================================================================
 * REMOVED in v5: HAPTIC_NONCE (v4: bits 0-2)
 * REMOVED in v5: HAPTIC_TARGET_POSITION (v4: bits 13-20)
 */

// Haptic mode: 3 bits at position 0
#define HAPTIC_MODE_bp              (0)
#define HAPTIC_MODE_bs              (3)
#define HAPTIC_MODE_bm              (((1U << HAPTIC_MODE_bs) - 1) << HAPTIC_MODE_bp)

// Detent count: 4 bits at position 3
#define HAPTIC_DETENT_COUNT_bp      (3)
#define HAPTIC_DETENT_COUNT_bs      (4)
#define HAPTIC_DETENT_COUNT_bm      (((1U << HAPTIC_DETENT_COUNT_bs) - 1) << HAPTIC_DETENT_COUNT_bp)

// Detent strength: 3 bits at position 7
#define HAPTIC_DETENT_STRENGTH_bp   (7)
#define HAPTIC_DETENT_STRENGTH_bs   (3)
#define HAPTIC_DETENT_STRENGTH_bm   (((1U << HAPTIC_DETENT_STRENGTH_bs) - 1) << HAPTIC_DETENT_STRENGTH_bp)

// Reserved: 6 bits at position 10-15 (must be 0)