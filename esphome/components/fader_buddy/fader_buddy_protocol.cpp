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

#include "fader_buddy_protocol.h"

#include <stdio.h>

#include "bootloader_protocol.h"
#include "fader_buddy_platform.h"

namespace faderbuddy {

void logf(Platform &platform, LogLevel level, const char *format, ...) {
  va_list args;
  va_start(args, format);
  platform.log(level, format, args);
  va_end(args);
}

FaderState decode_state(uint32_t raw) {
  FaderState s;
  s.raw = raw;
  s.mode = static_cast<Mode>((raw & STATE_MODE_bm) >> STATE_MODE_bp);
  s.touch = ((raw & STATE_TOUCH_bm) >> STATE_TOUCH_bp) != 0;
  s.active_layer = (uint8_t) ((raw & STATE_ACTIVE_LAYER_bm) >> STATE_ACTIVE_LAYER_bp);
  s.position = (uint8_t) ((raw & STATE_POSITION_bm) >> STATE_POSITION_bp);
  s.position_nonce = (uint8_t) ((raw & STATE_POSITION_NONCE_bm) >> STATE_POSITION_NONCE_bp);
  s.raw_adc = (uint16_t) ((raw & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp);
  s.double_tap_nonce = (uint8_t) ((raw & STATE_DOUBLE_TAP_NONCE_bm) >> STATE_DOUBLE_TAP_NONCE_bp);
  return s;
}

MotorCal decode_motor_cal(const uint8_t b[MOTOR_CAL_LEN]) {
  MotorCal c;
  c.valid = b[0] != 0;
  c.breakaway_rising = b[1];
  c.breakaway_falling = b[2];
  c.k_rising = b[3];
  c.k_falling = b[4];
  c.v_jump_rising = be16(b + 5);
  c.v_jump_falling = be16(b + 7);
  c.vel_min = be16(b + 9);
  c.deadband = b[11];
  return c;
}

uint16_t encode_haptic_config(uint8_t mode, uint8_t detent_count, uint8_t detent_strength) {
  uint16_t config = 0;
  config |= ((uint16_t) mode << HAPTIC_MODE_bp) & HAPTIC_MODE_bm;
  config |= ((uint16_t) detent_count << HAPTIC_DETENT_COUNT_bp) & HAPTIC_DETENT_COUNT_bm;
  config |= ((uint16_t) detent_strength << HAPTIC_DETENT_STRENGTH_bp) & HAPTIC_DETENT_STRENGTH_bm;
  return config;
}

uint16_t normalize_fw_version(uint16_t raw) { return raw == 0 ? FW_VERSION_NONE : raw; }

bool fw_version_known(uint16_t version) { return version != FW_VERSION_NONE && version != 0; }

bool fw_supports_move_speed(uint16_t version) {
  return fw_version_known(version) && version >= FW_VERSION_MOVE_SPEED;
}

bool fw_supports_motor_cal(uint16_t version) {
  return fw_version_known(version) && version >= FW_VERSION_MOTOR_CAL;
}

bool fw_supports_bootloader_entry(uint16_t version) {
  return fw_version_known(version) && version >= FW_VERSION_BOOTLOADER_ENTRY;
}

void format_fw_version(uint16_t version, char *buf, size_t len) {
  if (!fw_version_known(version)) {
    snprintf(buf, len, "1.0");
  } else {
    snprintf(buf, len, "%d.%d", version >> 8, version & 0xFF);
  }
}

uint16_t image_crc16(const uint8_t *data, uint32_t len) {
  uint16_t crc = BL_CRC16_INIT;
  for (uint32_t i = 0; i < len; i++) {
    crc = bl_crc16_update(crc, data[i]);
  }
  return crc;
}

}  // namespace faderbuddy
