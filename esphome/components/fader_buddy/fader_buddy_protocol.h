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

/*
 * Pure encode/decode of the wire formats in i2c_data.h and the version-gated
 * feature rules every host has to apply. No I/O, so it is unit-tested natively.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "i2c_data.h"

namespace faderbuddy {

static const uint8_t NUM_LAYERS = 8;
static const size_t SERIAL_NUMBER_LEN = 10;
static const size_t MOTOR_CAL_LEN = 12;

// REG_STATE, unpacked.
struct FaderState {
  uint32_t raw;
  Mode mode;
  bool touch;
  uint8_t active_layer;
  uint8_t position;  // hardware position, 0-255 (not inverted)
  uint8_t position_nonce;
  uint16_t raw_adc;
  uint8_t double_tap_nonce;
};
FaderState decode_state(uint32_t raw);

// REG_MOTOR_CAL, unpacked. See the register description in i2c_data.h.
struct MotorCal {
  bool valid;  // false: the fader is running compiled-in defaults
  uint8_t breakaway_rising;
  uint8_t breakaway_falling;
  uint8_t k_rising;
  uint8_t k_falling;
  uint16_t v_jump_rising;
  uint16_t v_jump_falling;
  uint16_t vel_min;
  uint8_t deadband;
};
MotorCal decode_motor_cal(const uint8_t raw[MOTOR_CAL_LEN]);

uint16_t encode_haptic_config(uint8_t mode, uint8_t detent_count, uint8_t detent_strength);

// Big-endian helpers, matching every multi-byte register.
inline uint16_t be16(const uint8_t *b) { return (uint16_t) (((uint16_t) b[0] << 8) | b[1]); }
inline uint32_t be32(const uint8_t *b) {
  return ((uint32_t) b[0] << 24) | ((uint32_t) b[1] << 16) | ((uint32_t) b[2] << 8) | b[3];
}

// --- Firmware version rules ---
//
// REG_FW_VERSION reads back 0xFFFF (FW_VERSION_NONE) from firmware predating
// it, which compares numerically ABOVE every real version. Every feature check
// has to exclude it explicitly, so they live here rather than at call sites.
// A raw read of 0x0000 is treated as FW_VERSION_NONE too.
uint16_t normalize_fw_version(uint16_t raw);
bool fw_version_known(uint16_t version);
bool fw_supports_move_speed(uint16_t version);
bool fw_supports_motor_cal(uint16_t version);
bool fw_supports_bootloader_entry(uint16_t version);

// "1.4". buf should hold at least 8 bytes. FW_VERSION_NONE formats as "1.0",
// the last unversioned release, which is what it means.
void format_fw_version(uint16_t version, char *buf, size_t len);

// The application section CRC used by the bootloader's verify, over `len`
// bytes of `data` (bl_crc16_update from BL_CRC16_INIT).
uint16_t image_crc16(const uint8_t *data, uint32_t len);

}  // namespace faderbuddy
