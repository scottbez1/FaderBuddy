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
 * One fader's registers, application and bootloader both, as typed calls. No
 * policy: no version checks, no state, no logging. Callers that want the rules
 * (when the speed byte may be sent, when a bootloader can be entered) use
 * FaderBuddy, or the helpers in fader_buddy_protocol.h.
 *
 * Every call is a single transaction unless given `attempts` > 1, in which case
 * it is repeated with `retry_delay_ms` between tries. The bootloader stops
 * answering while it erases or writes flash, so its commands are normally
 * retried; the application's registers normally are not.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bootloader_protocol.h"
#include "fader_buddy_platform.h"
#include "fader_buddy_protocol.h"
#include "i2c_data.h"

namespace faderbuddy {

struct BootloaderStatus {
  uint8_t version;     // BL_BOOTLOADER_VERSION
  uint8_t status;      // BL_STATUS_*
  uint8_t last_error;  // BL_ERR_*
};

class FaderBuddyDevice {
 public:
  FaderBuddyDevice(Transport &transport, Platform &platform) : transport_(transport), platform_(platform) {}

  Transport &transport() const { return transport_; }
  Platform &platform() const { return platform_; }

  // --- Generic transactions, with optional retry ---
  bool write(const uint8_t *data, size_t len, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;
  bool read(uint8_t *out, size_t len, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;
  bool write_read(const uint8_t *data, size_t len, uint8_t *out, size_t out_len, uint8_t attempts = 1,
                  uint32_t retry_delay_ms = 0) const;
  bool read_register(uint8_t reg, uint8_t *out, size_t len, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;
  bool write_command(uint8_t reg, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;

  // --- "Who are you": register 0x00 ---
  // The application answers its protocol version (I2C_PROTOCOL_VERSION); a
  // resident bootloader answers BL_VERSION_MARKER.
  bool read_probe(uint8_t &value, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;

  // --- Application registers ---
  bool read_state(uint32_t &raw) const;
  bool read_state(FaderState &state) const;
  bool read_uptime(uint32_t &uptime_ms) const;
  bool read_touch_raw(uint16_t &value) const;
  bool read_touch_delta(int16_t &value) const;
  bool read_touch_reference(uint16_t &value) const;
  bool read_touch_recal_count(uint16_t &value) const;
  bool read_serial_number(uint8_t out[SERIAL_NUMBER_LEN]) const;
  // Raw register value: FW_VERSION_NONE (0xFFFF) from firmware predating it,
  // which still ACKs. See normalize_fw_version().
  bool read_fw_version(uint16_t &version, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;
  bool read_motor_cal(uint8_t out[MOTOR_CAL_LEN]) const;
  bool read_motor_cal(MotorCal &cal) const;
  bool read_active_layer(uint8_t &layer) const;
  bool write_active_layer(uint8_t layer, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;
  bool read_layer_target(uint8_t layer, uint8_t &position) const;
  // Three-byte form: leaves the layer's stored speed in place.
  bool write_layer_target(uint8_t layer, uint8_t position, uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;
  // Four-byte form. Firmware older than FW_VERSION_MOVE_SPEED drops this
  // write entirely - check fw_supports_move_speed() first.
  bool write_layer_target(uint8_t layer, uint8_t position, uint8_t speed, uint8_t attempts,
                          uint32_t retry_delay_ms) const;
  bool write_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength,
                                 uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const;
  bool calibrate_touch() const { return write_command(REG_CAL_TOUCH); }
  bool clear_error() const { return write_command(REG_CLEAR_ERROR); }
  bool self_calibrate(uint8_t attempts = 1, uint32_t retry_delay_ms = 0) const {
    return write_command(REG_SELF_CAL, attempts, retry_delay_ms);
  }
  // Ignored by firmware older than FW_VERSION_BOOTLOADER_ENTRY.
  bool enter_bootloader(uint8_t attempts = 3, uint32_t retry_delay_ms = 5) const;

  // --- Bootloader commands (only meaningful while it is resident) ---
  bool bl_get_status(BootloaderStatus &status, uint8_t attempts = 3, uint32_t retry_delay_ms = 5) const;
  // Issues the erase only. The target then stops answering for a few hundred
  // ms; poll bl_get_status() until it comes back.
  bool bl_request_erase_app(uint8_t attempts = 3, uint32_t retry_delay_ms = 5) const;
  bool bl_set_page_addr(uint16_t flash_addr, uint8_t attempts = 3, uint32_t retry_delay_ms = 5) const;
  // 16 data bytes; the frame CRC is added here.
  bool bl_send_frame(const uint8_t *data16, uint8_t attempts = 3, uint32_t retry_delay_ms = 5) const;
  // SET_PAGE_ADDR + the page's BL_FRAMES_PER_PAGE frames.
  bool bl_write_page(uint16_t flash_addr, const uint8_t *page_data, uint8_t attempts = 3,
                     uint32_t retry_delay_ms = 5) const;
  // Asks for a CRC over [flash_addr, flash_addr + len). The bootloader computes
  // it after releasing the bus - it cannot clock-stretch that long, the ESP32
  // caps stretching at ~13ms - and NAKs reads until done, so the answer is
  // collected by polling bl_poll_crc16().
  bool bl_request_crc16(uint16_t flash_addr, uint16_t len, uint8_t attempts = 3, uint32_t retry_delay_ms = 5) const;
  // One attempt; false while the target is still computing.
  bool bl_poll_crc16(uint16_t &crc) const;
  bool bl_run_app(uint8_t attempts = 3, uint32_t retry_delay_ms = 5) const;

 private:
  Transport &transport_;
  Platform &platform_;
};

}  // namespace faderbuddy
