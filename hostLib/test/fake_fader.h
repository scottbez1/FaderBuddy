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
 * A simulated fader for the native tests: enough of the application's
 * registers and of the I2C bootloader (firmware/src/bootloader/bootloader.c)
 * to drive the host library end to end, including the bootloader's NAK-while-
 * busy behaviour, and hooks to inject the failures the updater must survive.
 */

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "bootloader_protocol.h"
#include "fader_buddy_platform.h"
#include "i2c_data.h"

namespace fbtest {

using faderbuddy::LogLevel;

// Time only moves when the code under test asks for it (or delays), so the
// tests are deterministic and polling loops always terminate.
class FakePlatform : public faderbuddy::Platform {
 public:
  uint32_t now = 1000;
  bool echo_logs = false;
  std::vector<std::string> logs;

  uint32_t millis() override { return now++; }
  void delay_ms(uint32_t ms) override { now += ms; }
  void log(LogLevel level, const char *format, va_list args) override {
    char buf[512];
    vsnprintf(buf, sizeof(buf), format, args);
    logs.push_back(buf);
    if (echo_logs) {
      printf("[%d] %s\n", (int) level, buf);
    }
  }
  bool logged(const char *needle) const {
    for (const auto &l : logs) {
      if (l.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }
};

class FakeFader : public faderbuddy::Transport {
 public:
  enum Mode { ABSENT, APP, BOOTLOADER };

  explicit FakeFader(FakePlatform &platform) : platform_(platform) {
    memset(flash, 0xFF, sizeof(flash));
    for (int i = 0; i < 10; i++) {
      serial[i] = (uint8_t) (0xA0 + i);
    }
    memset(motor_cal, 0, sizeof(motor_cal));
  }

  // --- Application state ---
  Mode mode = APP;
  uint8_t protocol_version = I2C_PROTOCOL_VERSION;
  uint16_t fw_version = 0x0105;
  uint32_t state = (uint32_t) MODE_INPUT_IDLE << STATE_MODE_bp;
  uint8_t serial[10];
  uint8_t motor_cal[12];
  uint8_t active_layer = 0;

  // --- Bootloader state ---
  uint8_t flash[BL_FLASH_SIZE];
  uint8_t last_error = BL_ERR_NONE;
  std::vector<uint16_t> page_write_order;  // flash addresses, in write order
  int erase_count = 0;

  // --- Fault injection ---
  int absent_probes_remaining = 0;     // NAK this many transactions, then appear
  uint16_t corrupt_page_addr = 0;       // flip a bit when this page is written...
  bool corrupt_page = false;            // ...if set
  bool reject_enter = false;            // app ignores REG_ENTER_BOOTLOADER
  bool app_reports_wrong_version = false;

  // --- Observation ---
  std::vector<std::vector<uint8_t>> app_writes;  // every write the app received
  int transactions = 0;

  std::vector<uint8_t> last_app_write(uint8_t reg) const {
    for (auto it = app_writes.rbegin(); it != app_writes.rend(); ++it) {
      if (!it->empty() && (*it)[0] == reg) {
        return *it;
      }
    }
    return {};
  }

  void set_state(::Mode m, bool touch, uint8_t layer, uint8_t position, uint8_t position_nonce = 0,
                 uint8_t double_tap_nonce = 0) {
    state = ((uint32_t) m << STATE_MODE_bp) | ((uint32_t) touch << STATE_TOUCH_bp) |
            ((uint32_t) layer << STATE_ACTIVE_LAYER_bp) | ((uint32_t) position << STATE_POSITION_bp) |
            ((uint32_t) position_nonce << STATE_POSITION_NONCE_bp) |
            ((uint32_t) double_tap_nonce << STATE_DOUBLE_TAP_NONCE_bp);
  }

  bool app_section_blank() const {
    for (uint32_t i = BL_APP_START; i < BL_FLASH_SIZE; i++) {
      if (flash[i] != 0xFF) return false;
    }
    return true;
  }
  bool reset_vector_blank() const { return flash[BL_APP_START] == 0xFF && flash[BL_APP_START + 1] == 0xFF; }

  // --- Transport ---
  bool write(const uint8_t *data, size_t len) override {
    if (!present_()) return false;
    std::vector<uint8_t> w(data, data + len);
    if (mode == APP) {
      app_writes.push_back(w);
      return app_write_(w);
    }
    return bl_write_(w);
  }

  bool read(uint8_t *data, size_t len) override {
    if (!present_()) return false;
    if (mode == BOOTLOADER) {
      // Bare read: the CRC result, once computed.
      if (crc_pending_) {
        if ((int32_t) (platform_.now - crc_ready_at_) < 0) return false;  // still computing: NAK
        uint8_t out[3] = {BL_BOOTLOADER_VERSION, (uint8_t) (crc_result_ >> 8), (uint8_t) (crc_result_ & 0xFF)};
        memcpy(data, out, len < 3 ? len : 3);
        return true;
      }
      memset(data, 0xFF, len);
      return true;
    }
    memset(data, 0xFF, len);
    return true;
  }

  bool write_read(const uint8_t *w, size_t wlen, uint8_t *r, size_t rlen) override {
    if (!present_()) return false;
    memset(r, 0xFF, rlen);  // undriven bus reads high
    if (wlen == 0) return false;
    if (mode == BOOTLOADER) {
      if (busy_()) return false;
      if (w[0] == 0x00) {
        r[0] = BL_VERSION_MARKER;
      } else if (w[0] == BL_CMD_GET_STATUS) {
        uint8_t out[3] = {BL_BOOTLOADER_VERSION, (uint8_t) (reset_vector_blank() ? BL_STATUS_NO_APP : BL_STATUS_IDLE),
                          last_error};
        memcpy(r, out, rlen < 3 ? rlen : 3);
      }
      return true;
    }
    // Application register reads.
    switch (w[0]) {
      case REG_VERSION:
        r[0] = protocol_version;
        break;
      case REG_STATE:
        put32_(r, state);
        break;
      case REG_SERIAL:
        memcpy(r, serial, rlen < 10 ? rlen : 10);
        break;
      case REG_FW_VERSION:
        if (fw_version != 0xFFFF) {  // older firmware leaves the bus undriven
          r[0] = fw_version >> 8;
          r[1] = fw_version & 0xFF;
        }
        break;
      case REG_MOTOR_CAL:
        if (fw_version != 0xFFFF && fw_version >= FW_VERSION_MOTOR_CAL) memcpy(r, motor_cal, rlen < 12 ? rlen : 12);
        break;
      case REG_ACTIVE_LAYER:
        r[0] = active_layer;
        break;
      case REG_LAYER_TARGET:
        r[0] = (wlen >= 2) ? layer_targets_[w[1] & 7] : 0;
        break;
      default:
        break;
    }
    return true;
  }

 private:
  bool present_() {
    transactions++;
    if (mode == ABSENT) return false;
    if (absent_probes_remaining > 0) {
      absent_probes_remaining--;
      return false;
    }
    return true;
  }
  bool busy_() const { return (int32_t) (platform_.now - busy_until_) < 0; }
  static void put32_(uint8_t *r, uint32_t v) {
    r[0] = v >> 24;
    r[1] = v >> 16;
    r[2] = v >> 8;
    r[3] = v;
  }

  bool app_write_(const std::vector<uint8_t> &w) {
    switch (w[0]) {
      case REG_ENTER_BOOTLOADER: {
        // FW_VERSION 0 is the jig's deliberately unversioned build of current
        // firmware, which does support entry.
        bool too_old = fw_version == 0xFFFF || (fw_version != 0 && fw_version < FW_VERSION_BOOTLOADER_ENTRY);
        if (w.size() != 5 || reject_enter || too_old) {
          return true;  // ACKed, ignored
        }
        uint32_t magic = ((uint32_t) w[1] << 24) | ((uint32_t) w[2] << 16) | ((uint32_t) w[3] << 8) | w[4];
        if (magic == ENTER_BOOTLOADER_MAGIC) {
          mode = BOOTLOADER;
          busy_until_ = platform_.now + 30;  // resetting
        }
        return true;
      }
      case REG_ACTIVE_LAYER:
        if (w.size() >= 2) active_layer = w[1] & 7;
        return true;
      case REG_LAYER_TARGET:
        // Firmware predating the speed byte drops a 4-byte write entirely.
        if (w.size() == 4 && (fw_version == 0xFFFF || fw_version < FW_VERSION_MOVE_SPEED)) return true;
        if (w.size() >= 3) layer_targets_[w[1] & 7] = w[2];
        return true;
      default:
        return true;
    }
  }

  bool bl_write_(const std::vector<uint8_t> &w) {
    if (busy_()) return false;
    switch (w[0]) {
      case BL_CMD_ERASE_APP:
        memset(flash + BL_APP_START, 0xFF, BL_APP_SIZE);
        erase_count++;
        frames_ = -1;
        busy_until_ = platform_.now + 300;  // CPU stalled in NVM
        return true;
      case BL_CMD_SET_PAGE_ADDR:
        if (w.size() != 3) return true;
        page_addr_ = (uint16_t) ((w[1] << 8) | w[2]);
        if (page_addr_ < BL_APP_START || page_addr_ >= BL_FLASH_SIZE || page_addr_ % BL_PAGE_SIZE) {
          last_error = BL_ERR_ADDR_RANGE;
          frames_ = -1;
          return true;
        }
        frames_ = 0;
        return true;
      case BL_CMD_SEND_FRAME: {
        if (frames_ < 0) {
          last_error = BL_ERR_FRAME_SEQ;
          return true;
        }
        if (w.size() != 1 + BL_FRAME_DATA_LEN + 2) return true;
        uint16_t crc = BL_CRC16_INIT;
        for (uint32_t i = 0; i < BL_FRAME_DATA_LEN; i++) crc = bl_crc16_update(crc, w[1 + i]);
        uint16_t got = (uint16_t) ((w[17] << 8) | w[18]);
        if (crc != got) {
          last_error = BL_ERR_CRC;
          return true;
        }
        memcpy(page_buf_ + frames_ * BL_FRAME_DATA_LEN, &w[1], BL_FRAME_DATA_LEN);
        frames_++;
        if (frames_ == (int) BL_FRAMES_PER_PAGE) {
          if (corrupt_page && page_addr_ == corrupt_page_addr) page_buf_[7] ^= 0x01;
          memcpy(flash + page_addr_, page_buf_, BL_PAGE_SIZE);
          page_write_order.push_back(page_addr_);
          frames_ = -1;
          busy_until_ = platform_.now + 4;  // page erase+write stall
        }
        return true;
      }
      case BL_CMD_GET_VERSION_CRC16: {
        if (w.size() != 5) return true;
        uint16_t addr = (uint16_t) ((w[1] << 8) | w[2]);
        uint16_t len = (uint16_t) ((w[3] << 8) | w[4]);
        uint16_t crc = BL_CRC16_INIT;
        for (uint32_t i = 0; i < len && addr + i < BL_FLASH_SIZE; i++) crc = bl_crc16_update(crc, flash[addr + i]);
        crc_result_ = crc;
        crc_pending_ = true;
        crc_ready_at_ = platform_.now + 60;
        return true;
      }
      case BL_CMD_RUN_APP:
        if (!reset_vector_blank()) {
          mode = APP;
          crc_pending_ = false;
          fw_version = (uint16_t) ((flash[BL_APP_META_ADDR] << 8) | flash[BL_APP_META_ADDR + 1]);
          if (app_reports_wrong_version) fw_version ^= 0x0001;
          busy_until_ = platform_.now + 50;  // booting
        }
        return true;
      default:
        return true;
    }
  }

  FakePlatform &platform_;
  uint32_t busy_until_ = 0;
  int frames_ = -1;
  uint16_t page_addr_ = 0;
  uint8_t page_buf_[BL_PAGE_SIZE];
  bool crc_pending_ = false;
  uint32_t crc_ready_at_ = 0;
  uint16_t crc_result_ = 0;
  uint8_t layer_targets_[8] = {0};
};

// A plausible application image: deterministic bytes, a non-blank reset
// vector, and the FW_VERSION footer in its last two bytes.
inline std::vector<uint8_t> make_image(uint16_t fw_version, uint32_t length = BL_APP_SIZE) {
  std::vector<uint8_t> img(length);
  for (uint32_t i = 0; i < length; i++) img[i] = (uint8_t) ((i * 31 + 7) ^ (i >> 8));
  img[0] = 0x0C;  // rjmp-ish, anything but 0xFF
  img[1] = 0x94;
  img[length - 2] = fw_version >> 8;
  img[length - 1] = fw_version & 0xFF;
  return img;
}

}  // namespace fbtest
