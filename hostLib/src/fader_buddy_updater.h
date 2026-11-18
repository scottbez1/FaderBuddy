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
 * Installs an application image through the I2C bootloader - the one copy of
 * the update sequence, shared by every host. See ABOUT_I2C_BOOTLOADER.md.
 *
 * Non-blocking: start() it, then call tick() until it returns false. Each tick
 * either finishes its step and advances, polls something the target is still
 * busy with, or fails the run. Only page streaming takes real time per tick,
 * and that is capped at Options::write_budget_ms.
 *
 * Page 0 holds the reset vector, which the erase blanks. It is written last,
 * and only once every other page has verified, so an update that dies part-way
 * leaves the bootloader's app_is_valid() failing and the board resident in its
 * bootloader across resets, instead of booting a mix of new and old pages.
 * A failure after page 0 is written re-blanks it (best effort).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "fader_buddy_device.h"

namespace faderbuddy {

enum class UpdateStage : uint8_t {
  IDLE = 0,
  PROBE,           // app or bootloader? and the go/no-go checks
  WAIT_MARKER,     // after REG_ENTER_BOOTLOADER
  ERASE,           // status check, then issue the erase
  ERASE_WAIT,      // target is stalled in NVM, poll it back
  WRITE,           // stream pages 1..N-1 (not page 0), budgeted per tick
  BODY_CHECK,      // NVM status, then ask for the CRC of pages 1..N-1
  BODY_CRC_WAIT,   // poll for it; on a match, write page 0
  WRITE_CHECK,     // post-write NVM status, then ask for the whole-image CRC
  CRC_WAIT,        // poll for it
  WAIT_APP,        // after RUN_APP
  CONFIRM,         // REG_FW_VERSION matches the image
  DONE,
  FAILED,
};

enum class UpdateError : uint8_t {
  NONE = 0,
  BAD_IMAGE,                    // empty, or not a whole number of pages
  NOT_RESPONDING,               // no answer to the probe
  PREDATES_BOOTLOADER_ENTRY,    // app too old to enter its bootloader: UPDI only
  ENTER_FAILED,                 // REG_ENTER_BOOTLOADER write NAKed
  NO_MARKER,                    // never saw the bootloader after entering it
  NO_STATUS,                    // bootloader didn't answer GET_STATUS
  ERASE_FAILED,                 // erase command NAKed, or timed out
  NVM_ERROR,                    // bootloader recorded an NVM error (see nvm_error())
  WRITE_FAILED,                 // a page write NAKed
  CRC_READ_FAILED,              // CRC request NAKed or never answered
  CRC_MISMATCH,                 // body or whole-image CRC differs from the image
  RUN_FAILED,                   // RUN_APP NAKed
  APP_DID_NOT_START,            // no app protocol version after RUN_APP
  FW_VERSION_READ_FAILED,       // new app didn't answer REG_FW_VERSION
  FW_VERSION_MISMATCH,          // new app reports a different FW_VERSION
  CANCELLED,                    // cancel() called
};

const char *update_stage_name(UpdateStage stage);

struct FirmwareImage {
  const uint8_t *data;  // exact APPCODE bytes, starting at BL_APP_START
  uint32_t length;      // whole pages
  uint16_t crc16;       // image_crc16() over all `length` bytes
  uint16_t fw_version;  // what REG_FW_VERSION must report once it runs
};

class UpdateObserver {
 public:
  virtual ~UpdateObserver() = default;
  // Called on entering each stage, DONE and FAILED included.
  virtual void on_update_stage(UpdateStage stage) { (void) stage; }
  // Called after each slice of page writes. total counts page 0, which is
  // written last, so `written` reaches total - 1 at the end of WRITE and total
  // once page 0 is committed.
  virtual void on_update_pages_written(uint32_t written, uint32_t total) {
    (void) written;
    (void) total;
  }
};

class FirmwareUpdater {
 public:
  struct Options {
    // Treat "already running this version" as success without touching flash.
    // Off for the test jig, whose point is to exercise the install.
    bool skip_if_current = false;
    // Refuse a running app whose REG_FW_VERSION predates
    // FW_VERSION_BOOTLOADER_ENTRY, rather than send REG_ENTER_BOOTLOADER and
    // time out waiting for a bootloader that isn't there. Off only for the
    // test jig, whose "old" app is a deliberately unversioned (FW_VERSION 0)
    // build of firmware that does support entry.
    bool require_bootloader_entry_version = true;
    // How long one tick may spend streaming pages before yielding.
    uint32_t write_budget_ms = 20;
    // Attempts for each page/frame write and status read; the bootloader
    // NAKs while a page is being written to flash.
    uint8_t retry_attempts = 3;
    uint32_t retry_delay_ms = 5;
    // The opening probe is retried harder: a fader that has just been reset or
    // reflashed may still be booting.
    uint8_t probe_attempts = 5;
    uint32_t probe_retry_delay_ms = 20;
  };

  FirmwareUpdater(const FaderBuddyDevice &device, UpdateObserver *observer = nullptr)
      : device_(device), observer_(observer) {}

  void set_observer(UpdateObserver *observer) { observer_ = observer; }

  // Begin an install. Returns false (and ends FAILED, BAD_IMAGE) for an image
  // that isn't a whole number of pages. Starting while running restarts.
  bool start(const FirmwareImage &image, const Options &options);
  bool start(const FirmwareImage &image) { return start(image, Options()); }
  // Advance. Returns true while still running.
  bool tick();
  // Stop at once, with no further bus traffic. Ends FAILED, CANCELLED.
  void cancel();

  bool running() const { return stage_ != UpdateStage::IDLE && stage_ != UpdateStage::DONE && stage_ != UpdateStage::FAILED; }
  UpdateStage stage() const { return stage_; }
  bool succeeded() const { return stage_ == UpdateStage::DONE; }
  // DONE without installing anything: skip_if_current and already current.
  bool was_noop() const { return noop_; }
  UpdateError error() const { return error_; }
  // Human-readable reason; "" on success.
  const char *error_message() const { return error_message_; }
  // The NVM error code behind UpdateError::NVM_ERROR.
  uint8_t nvm_error() const { return nvm_error_; }
  // Which stage the failure happened in.
  UpdateStage failed_stage() const { return failed_stage_; }
  // Last known: is the fader sitting in its bootloader?
  bool bootloader_resident() const { return bootloader_resident_; }
  uint32_t pages_written() const { return pages_written_; }
  uint32_t pages_total() const { return pages_total_; }
  uint32_t started_at() const { return started_at_; }

 private:
  void enter_(UpdateStage stage, uint32_t timeout_ms = 0);
  void fail_(UpdateError error, const char *message);
  void fail_and_uncommit_(UpdateError error, const char *message);
  bool write_page_(uint32_t page);
  bool nvm_error_pending_(const char *when, bool uncommit);

  const FaderBuddyDevice &device_;
  UpdateObserver *observer_;
  FirmwareImage image_{nullptr, 0, 0, 0};
  Options options_;

  UpdateStage stage_{UpdateStage::IDLE};
  UpdateStage failed_stage_{UpdateStage::IDLE};
  UpdateError error_{UpdateError::NONE};
  char error_message_[64] = {0};
  uint8_t nvm_error_{0};
  bool noop_{false};
  bool bootloader_resident_{false};
  uint32_t deadline_{0};
  uint32_t started_at_{0};
  uint32_t next_page_{0};
  volatile uint32_t pages_written_{0};
  uint32_t pages_total_{0};
};

}  // namespace faderbuddy
