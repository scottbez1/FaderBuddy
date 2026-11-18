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

#include "fader_buddy_device.h"

namespace faderbuddy {

bool FaderBuddyDevice::write(const uint8_t *data, size_t len, uint8_t attempts, uint32_t retry_delay_ms) const {
  for (uint8_t a = 0; a < attempts; a++) {
    if (a > 0 && retry_delay_ms > 0) {
      platform_.delay_ms(retry_delay_ms);
    }
    if (transport_.write(data, len)) {
      return true;
    }
  }
  return false;
}

bool FaderBuddyDevice::read(uint8_t *out, size_t len, uint8_t attempts, uint32_t retry_delay_ms) const {
  for (uint8_t a = 0; a < attempts; a++) {
    if (a > 0 && retry_delay_ms > 0) {
      platform_.delay_ms(retry_delay_ms);
    }
    if (transport_.read(out, len)) {
      return true;
    }
  }
  return false;
}

bool FaderBuddyDevice::write_read(const uint8_t *data, size_t len, uint8_t *out, size_t out_len, uint8_t attempts,
                                  uint32_t retry_delay_ms) const {
  for (uint8_t a = 0; a < attempts; a++) {
    if (a > 0 && retry_delay_ms > 0) {
      platform_.delay_ms(retry_delay_ms);
    }
    if (transport_.write_read(data, len, out, out_len)) {
      return true;
    }
  }
  return false;
}

bool FaderBuddyDevice::read_register(uint8_t reg, uint8_t *out, size_t len, uint8_t attempts,
                                     uint32_t retry_delay_ms) const {
  return write_read(&reg, 1, out, len, attempts, retry_delay_ms);
}

bool FaderBuddyDevice::write_command(uint8_t reg, uint8_t attempts, uint32_t retry_delay_ms) const {
  return write(&reg, 1, attempts, retry_delay_ms);
}

bool FaderBuddyDevice::read_probe(uint8_t &value, uint8_t attempts, uint32_t retry_delay_ms) const {
  return read_register(REG_VERSION, &value, 1, attempts, retry_delay_ms);
}

bool FaderBuddyDevice::read_state(uint32_t &raw) const {
  uint8_t b[4];
  if (!read_register(REG_STATE, b, sizeof(b))) {
    return false;
  }
  raw = be32(b);
  return true;
}

bool FaderBuddyDevice::read_state(FaderState &state) const {
  uint32_t raw;
  if (!read_state(raw)) {
    return false;
  }
  state = decode_state(raw);
  return true;
}

bool FaderBuddyDevice::read_uptime(uint32_t &uptime_ms) const {
  uint8_t b[4];
  if (!read_register(REG_UPTIME, b, sizeof(b))) {
    return false;
  }
  uptime_ms = be32(b);
  return true;
}

bool FaderBuddyDevice::read_touch_raw(uint16_t &value) const {
  uint8_t b[2];
  if (!read_register(REG_TOUCH_RAW, b, sizeof(b))) {
    return false;
  }
  value = be16(b);
  return true;
}

bool FaderBuddyDevice::read_touch_delta(int16_t &value) const {
  uint8_t b[2];
  if (!read_register(REG_TOUCH_DELTA, b, sizeof(b))) {
    return false;
  }
  value = (int16_t) be16(b);
  return true;
}

bool FaderBuddyDevice::read_touch_reference(uint16_t &value) const {
  uint8_t b[2];
  if (!read_register(REG_TOUCH_REF, b, sizeof(b))) {
    return false;
  }
  value = be16(b);
  return true;
}

bool FaderBuddyDevice::read_touch_recal_count(uint16_t &value) const {
  uint8_t b[2];
  if (!read_register(REG_TOUCH_RECAL, b, sizeof(b))) {
    return false;
  }
  value = be16(b);
  return true;
}

bool FaderBuddyDevice::read_serial_number(uint8_t out[SERIAL_NUMBER_LEN]) const {
  return read_register(REG_SERIAL, out, SERIAL_NUMBER_LEN);
}

bool FaderBuddyDevice::read_fw_version(uint16_t &version, uint8_t attempts, uint32_t retry_delay_ms) const {
  uint8_t b[2] = {0xFF, 0xFF};
  if (!read_register(REG_FW_VERSION, b, sizeof(b), attempts, retry_delay_ms)) {
    return false;
  }
  version = be16(b);
  return true;
}

bool FaderBuddyDevice::read_motor_cal(uint8_t out[MOTOR_CAL_LEN]) const {
  return read_register(REG_MOTOR_CAL, out, MOTOR_CAL_LEN);
}

bool FaderBuddyDevice::read_motor_cal(MotorCal &cal) const {
  uint8_t b[MOTOR_CAL_LEN] = {0};
  if (!read_motor_cal(b)) {
    return false;
  }
  cal = decode_motor_cal(b);
  return true;
}

bool FaderBuddyDevice::read_active_layer(uint8_t &layer) const {
  uint8_t b = 0;
  if (!read_register(REG_ACTIVE_LAYER, &b, 1)) {
    return false;
  }
  layer = b & 0x07;
  return true;
}

bool FaderBuddyDevice::write_active_layer(uint8_t layer, uint8_t attempts, uint32_t retry_delay_ms) const {
  uint8_t b[] = {REG_ACTIVE_LAYER, layer};
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::read_layer_target(uint8_t layer, uint8_t &position) const {
  uint8_t w[] = {REG_LAYER_TARGET, layer};
  return write_read(w, sizeof(w), &position, 1);
}

bool FaderBuddyDevice::write_layer_target(uint8_t layer, uint8_t position, uint8_t attempts,
                                          uint32_t retry_delay_ms) const {
  uint8_t b[] = {REG_LAYER_TARGET, layer, position};
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::write_layer_target(uint8_t layer, uint8_t position, uint8_t speed, uint8_t attempts,
                                          uint32_t retry_delay_ms) const {
  uint8_t b[] = {REG_LAYER_TARGET, layer, position, speed};
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::write_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count,
                                                 uint8_t detent_strength, uint8_t attempts,
                                                 uint32_t retry_delay_ms) const {
  uint16_t config = encode_haptic_config(mode, detent_count, detent_strength);
  uint8_t b[] = {REG_LAYER_HAPTIC_CONFIG, layer, (uint8_t) (config >> 8), (uint8_t) (config & 0xFF)};
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::enter_bootloader(uint8_t attempts, uint32_t retry_delay_ms) const {
  uint8_t b[5] = {
      REG_ENTER_BOOTLOADER,
      (uint8_t) (ENTER_BOOTLOADER_MAGIC >> 24),
      (uint8_t) (ENTER_BOOTLOADER_MAGIC >> 16),
      (uint8_t) (ENTER_BOOTLOADER_MAGIC >> 8),
      (uint8_t) (ENTER_BOOTLOADER_MAGIC),
  };
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::bl_get_status(BootloaderStatus &status, uint8_t attempts, uint32_t retry_delay_ms) const {
  uint8_t out[3];
  if (!read_register(BL_CMD_GET_STATUS, out, sizeof(out), attempts, retry_delay_ms)) {
    return false;
  }
  status.version = out[0];
  status.status = out[1];
  status.last_error = out[2];
  return true;
}

bool FaderBuddyDevice::bl_request_erase_app(uint8_t attempts, uint32_t retry_delay_ms) const {
  return write_command(BL_CMD_ERASE_APP, attempts, retry_delay_ms);
}

bool FaderBuddyDevice::bl_set_page_addr(uint16_t flash_addr, uint8_t attempts, uint32_t retry_delay_ms) const {
  uint8_t b[3] = {BL_CMD_SET_PAGE_ADDR, (uint8_t) (flash_addr >> 8), (uint8_t) (flash_addr & 0xFF)};
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::bl_send_frame(const uint8_t *data16, uint8_t attempts, uint32_t retry_delay_ms) const {
  uint8_t b[1 + BL_FRAME_DATA_LEN + 2];
  b[0] = BL_CMD_SEND_FRAME;
  uint16_t crc = BL_CRC16_INIT;
  for (uint8_t i = 0; i < BL_FRAME_DATA_LEN; i++) {
    b[1 + i] = data16[i];
    crc = bl_crc16_update(crc, data16[i]);
  }
  b[1 + BL_FRAME_DATA_LEN] = (uint8_t) (crc >> 8);
  b[1 + BL_FRAME_DATA_LEN + 1] = (uint8_t) (crc & 0xFF);
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::bl_write_page(uint16_t flash_addr, const uint8_t *page_data, uint8_t attempts,
                                     uint32_t retry_delay_ms) const {
  if (!bl_set_page_addr(flash_addr, attempts, retry_delay_ms)) {
    return false;
  }
  for (uint8_t f = 0; f < BL_FRAMES_PER_PAGE; f++) {
    if (!bl_send_frame(page_data + f * BL_FRAME_DATA_LEN, attempts, retry_delay_ms)) {
      return false;
    }
  }
  return true;
}

bool FaderBuddyDevice::bl_request_crc16(uint16_t flash_addr, uint16_t len, uint8_t attempts,
                                        uint32_t retry_delay_ms) const {
  uint8_t b[5] = {BL_CMD_GET_VERSION_CRC16, (uint8_t) (flash_addr >> 8), (uint8_t) (flash_addr & 0xFF),
                  (uint8_t) (len >> 8), (uint8_t) (len & 0xFF)};
  return write(b, sizeof(b), attempts, retry_delay_ms);
}

bool FaderBuddyDevice::bl_poll_crc16(uint16_t &crc) const {
  uint8_t out[3];
  if (!read(out, sizeof(out))) {
    return false;
  }
  crc = be16(out + 1);
  return true;
}

bool FaderBuddyDevice::bl_run_app(uint8_t attempts, uint32_t retry_delay_ms) const {
  return write_command(BL_CMD_RUN_APP, attempts, retry_delay_ms);
}

}  // namespace faderbuddy
