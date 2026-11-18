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

#include "fader_buddy_updater.h"

#include <stdio.h>
#include <string.h>

namespace faderbuddy {

const char *update_stage_name(UpdateStage stage) {
  switch (stage) {
    case UpdateStage::IDLE:
      return "idle";
    case UpdateStage::PROBE:
      return "probe";
    case UpdateStage::WAIT_MARKER:
      return "entering bootloader";
    case UpdateStage::ERASE:
    case UpdateStage::ERASE_WAIT:
      return "erasing";
    case UpdateStage::WRITE:
      return "writing";
    case UpdateStage::BODY_CHECK:
    case UpdateStage::BODY_CRC_WAIT:
    case UpdateStage::WRITE_CHECK:
    case UpdateStage::CRC_WAIT:
      return "verifying";
    case UpdateStage::WAIT_APP:
    case UpdateStage::CONFIRM:
      return "starting app";
    case UpdateStage::DONE:
      return "done";
    case UpdateStage::FAILED:
      return "failed";
  }
  return "?";
}

bool FirmwareUpdater::start(const FirmwareImage &image, const Options &options) {
  image_ = image;
  options_ = options;
  error_ = UpdateError::NONE;
  error_message_[0] = '\0';
  nvm_error_ = 0;
  noop_ = false;
  failed_stage_ = UpdateStage::IDLE;
  next_page_ = 0;
  pages_written_ = 0;
  pages_total_ = image.length / BL_PAGE_SIZE;
  started_at_ = device_.platform().millis();

  if (image.data == nullptr || image.length == 0 || (image.length % BL_PAGE_SIZE) != 0) {
    stage_ = UpdateStage::PROBE;  // so failed_stage() says where
    fail_(UpdateError::BAD_IMAGE, "bad image length");
    return false;
  }
  enter_(UpdateStage::PROBE);
  return true;
}

void FirmwareUpdater::cancel() {
  if (running()) {
    fail_(UpdateError::CANCELLED, "cancelled");
  }
}

void FirmwareUpdater::enter_(UpdateStage stage, uint32_t timeout_ms) {
  stage_ = stage;
  deadline_ = device_.platform().millis() + timeout_ms;
  if (observer_ != nullptr) {
    observer_->on_update_stage(stage);
  }
}

void FirmwareUpdater::fail_(UpdateError error, const char *message) {
  failed_stage_ = stage_;
  error_ = error;
  snprintf(error_message_, sizeof(error_message_), "%s", message);
  enter_(UpdateStage::FAILED);
}

// Fail an update after page 0 (the reset vector) has been written. From that
// point app_is_valid() passes, so a fader left in its bootloader would boot
// whatever is in flash at its next reset. Before RUN_APP that image is not
// known good, so blank the reset vector again: the fader then stays resident
// until a retry succeeds. Best effort - if the erase itself is lost, this is
// no worse than not trying.
void FirmwareUpdater::fail_and_uncommit_(UpdateError error, const char *message) {
  if (!device_.bl_request_erase_app()) {
    logf(device_.platform(), LogLevel::WARN, "update_firmware: could not re-blank the reset vector after failure");
  }
  fail_(error, message);
}

bool FirmwareUpdater::write_page_(uint32_t page) {
  if (!device_.bl_write_page((uint16_t) (BL_APP_START + page * BL_PAGE_SIZE), image_.data + page * BL_PAGE_SIZE,
                             options_.retry_attempts, options_.retry_delay_ms)) {
    return false;
  }
  pages_written_ = pages_written_ + 1;
  return true;
}

// Fails the run if the bootloader recorded an NVM error. A status read that
// itself fails is not treated as one - the CRC check that follows catches a
// bad write either way.
bool FirmwareUpdater::nvm_error_pending_(const char *when, bool uncommit) {
  BootloaderStatus st;
  if (device_.bl_get_status(st, options_.retry_attempts, options_.retry_delay_ms) && st.last_error != BL_ERR_NONE) {
    nvm_error_ = st.last_error;
    char buf[40];
    snprintf(buf, sizeof(buf), "nvm err=%u after %s", st.last_error, when);
    if (uncommit) {
      fail_and_uncommit_(UpdateError::NVM_ERROR, buf);
    } else {
      fail_(UpdateError::NVM_ERROR, buf);
    }
    return true;
  }
  return false;
}

bool FirmwareUpdater::tick() {
  Platform &platform = device_.platform();
  bool timed_out = (int32_t) (platform.millis() - deadline_) >= 0;

  switch (stage_) {
    case UpdateStage::IDLE:
    case UpdateStage::DONE:
    case UpdateStage::FAILED:
      return false;

    case UpdateStage::PROBE: {
      // Register 0x00 is the universal "who are you" probe: a protocol version
      // means the app is running (and REG_FW_VERSION is meaningful);
      // BL_VERSION_MARKER means the bootloader is already resident. Probe that
      // first - REG_FW_VERSION is app-only and undefined in the bootloader.
      uint8_t probe;
      if (!device_.read_probe(probe, options_.probe_attempts, options_.probe_retry_delay_ms)) {
        fail_(UpdateError::NOT_RESPONDING, "device not responding");
        return false;
      }
      if (probe == BL_VERSION_MARKER) {
        bootloader_resident_ = true;
        logf(platform, LogLevel::INFO, "update_firmware: updating to v%u.%u", image_.fw_version >> 8,
             image_.fw_version & 0xFF);
        enter_(UpdateStage::ERASE);
        return true;
      }
      bootloader_resident_ = false;

      uint16_t current = 0;
      bool have_version = device_.read_fw_version(current, options_.retry_attempts, options_.retry_delay_ms);
      if (have_version && options_.skip_if_current && current == image_.fw_version) {
        // Not a failure: the fader is already where the caller wants it.
        logf(platform, LogLevel::INFO, "update_firmware: already at v%u.%u, nothing to do", current >> 8,
             current & 0xFF);
        noop_ = true;
        enter_(UpdateStage::DONE);
        return false;
      }

      // Firmware predating FW_VERSION_BOOTLOADER_ENTRY ignores
      // REG_ENTER_BOOTLOADER, and has no bootloader behind it to reach anyway
      // (installing one is a fuse write, UPDI only). Refuse rather than write
      // the magic and time out waiting for a marker that will never appear.
      if (have_version && options_.require_bootloader_entry_version && !fw_supports_bootloader_entry(current)) {
        logf(platform, LogLevel::ERROR,
             "update_firmware: firmware v%u.%u predates I2C bootloader entry (needs >= v%u.%u); one-time UPDI "
             "migration required",
             current >> 8, current & 0xFF, FW_VERSION_BOOTLOADER_ENTRY >> 8, FW_VERSION_BOOTLOADER_ENTRY & 0xFF);
        fail_(UpdateError::PREDATES_BOOTLOADER_ENTRY, "firmware predates bootloader entry");
        return false;
      }

      logf(platform, LogLevel::INFO, "update_firmware: updating to v%u.%u", image_.fw_version >> 8,
           image_.fw_version & 0xFF);
      if (!device_.enter_bootloader()) {
        fail_(UpdateError::ENTER_FAILED, "enter bootloader cmd failed");
        return false;
      }
      enter_(UpdateStage::WAIT_MARKER, 2000);
      return true;
    }

    case UpdateStage::WAIT_MARKER: {
      uint8_t v;
      if (device_.read_probe(v) && v == BL_VERSION_MARKER) {
        bootloader_resident_ = true;
        enter_(UpdateStage::ERASE);
        return true;
      }
      if (timed_out) {
        fail_(UpdateError::NO_MARKER, "no bootloader marker");
        return false;
      }
      return true;
    }

    case UpdateStage::ERASE: {
      BootloaderStatus st;
      if (!device_.bl_get_status(st, options_.retry_attempts, options_.retry_delay_ms)) {
        fail_(UpdateError::NO_STATUS, "no bootloader status");
        return false;
      }
      if (!device_.bl_request_erase_app()) {
        fail_(UpdateError::ERASE_FAILED, "erase cmd failed");
        return false;
      }
      // The erase stalls the target CPU for hundreds of ms; it simply stops
      // answering until it's done.
      enter_(UpdateStage::ERASE_WAIT, 3000);
      return true;
    }

    case UpdateStage::ERASE_WAIT: {
      BootloaderStatus st;
      if (device_.bl_get_status(st, options_.retry_attempts, options_.retry_delay_ms)) {
        if (st.last_error != BL_ERR_NONE) {
          nvm_error_ = st.last_error;
          char buf[40];
          snprintf(buf, sizeof(buf), "nvm err=%u after erase", st.last_error);
          fail_(UpdateError::NVM_ERROR, buf);
          return false;
        }
        // Page 0 goes last - see the header.
        next_page_ = 1;
        enter_(UpdateStage::WRITE);
        return true;
      }
      if (timed_out) {
        fail_(UpdateError::ERASE_FAILED, "erase timed out");
        return false;
      }
      return true;
    }

    case UpdateStage::WRITE: {
      uint32_t slice_start = platform.millis();
      // Stream pages until this tick's budget is used up, then yield so the
      // rest of the host keeps running.
      while (next_page_ < pages_total_ && platform.millis() - slice_start < options_.write_budget_ms) {
        if (!write_page_(next_page_)) {
          fail_(UpdateError::WRITE_FAILED, "page write failed");
          return false;
        }
        next_page_++;
      }
      if (observer_ != nullptr) {
        observer_->on_update_pages_written(pages_written_, pages_total_);
      }
      if (next_page_ >= pages_total_) {
        enter_(UpdateStage::BODY_CHECK);
      }
      return true;
    }

    case UpdateStage::BODY_CHECK: {
      // A one-page image has no body to verify; go straight to page 0.
      if (pages_total_ <= 1) {
        enter_(UpdateStage::BODY_CRC_WAIT, 1000);
        return true;
      }
      if (nvm_error_pending_("write", false)) {
        return false;
      }
      if (!device_.bl_request_crc16(BL_APP_START + BL_PAGE_SIZE, (uint16_t) (image_.length - BL_PAGE_SIZE))) {
        fail_(UpdateError::CRC_READ_FAILED, "crc request failed");
        return false;
      }
      enter_(UpdateStage::BODY_CRC_WAIT, 1000);
      return true;
    }

    case UpdateStage::BODY_CRC_WAIT: {
      if (pages_total_ > 1) {
        uint16_t crc;
        if (!device_.bl_poll_crc16(crc)) {
          if (timed_out) {
            fail_(UpdateError::CRC_READ_FAILED, "crc read failed");
            return false;
          }
          return true;
        }
        // The image's CRC covers the whole image, so the body's is computed
        // here. ~16 KB of bitwise CRC is well under a millisecond on an ESP32.
        uint16_t expected = image_crc16(image_.data + BL_PAGE_SIZE, image_.length - BL_PAGE_SIZE);
        if (crc != expected) {
          // Page 0 is still blank, so the fader stays in its bootloader even
          // across a power cycle, and can simply be updated again.
          char buf[48];
          snprintf(buf, sizeof(buf), "CRC mismatch got=0x%04X exp=0x%04X", crc, expected);
          fail_(UpdateError::CRC_MISMATCH, buf);
          return false;
        }
      }
      // The rest of the image is verified: commit it by writing the reset vector.
      if (!write_page_(0)) {
        fail_and_uncommit_(UpdateError::WRITE_FAILED, "page write failed");
        return false;
      }
      if (observer_ != nullptr) {
        observer_->on_update_pages_written(pages_written_, pages_total_);
      }
      enter_(UpdateStage::WRITE_CHECK);
      return true;
    }

    case UpdateStage::WRITE_CHECK: {
      if (nvm_error_pending_("write", true)) {
        return false;
      }
      if (!device_.bl_request_crc16(BL_APP_START, (uint16_t) image_.length)) {
        fail_and_uncommit_(UpdateError::CRC_READ_FAILED, "crc request failed");
        return false;
      }
      enter_(UpdateStage::CRC_WAIT, 1000);
      return true;
    }

    case UpdateStage::CRC_WAIT: {
      uint16_t crc;
      if (device_.bl_poll_crc16(crc)) {
        if (crc != image_.crc16) {
          char buf[48];
          snprintf(buf, sizeof(buf), "CRC mismatch got=0x%04X exp=0x%04X", crc, image_.crc16);
          fail_and_uncommit_(UpdateError::CRC_MISMATCH, buf);
          return false;
        }
        // Not fail_and_uncommit_() from here on: the image verified, and RUN_APP
        // may have landed despite the error, in which case an ERASE_APP opcode
        // would be a register write to the running application.
        if (!device_.bl_run_app()) {
          fail_(UpdateError::RUN_FAILED, "run app cmd failed");
          return false;
        }
        enter_(UpdateStage::WAIT_APP, 2000);
        return true;
      }
      if (timed_out) {
        fail_and_uncommit_(UpdateError::CRC_READ_FAILED, "crc read failed");
        return false;
      }
      return true;
    }

    case UpdateStage::WAIT_APP: {
      uint8_t v;
      if (device_.read_probe(v) && v != BL_VERSION_MARKER && v != 0xFF && v != 0x00) {
        bootloader_resident_ = false;
        enter_(UpdateStage::CONFIRM);
        return true;
      }
      if (timed_out) {
        fail_(UpdateError::APP_DID_NOT_START, "app did not start");
        return false;
      }
      return true;
    }

    case UpdateStage::CONFIRM: {
      uint16_t fw;
      if (!device_.read_fw_version(fw, options_.retry_attempts, options_.retry_delay_ms)) {
        fail_(UpdateError::FW_VERSION_READ_FAILED, "no fw version after update");
        return false;
      }
      if (fw != image_.fw_version) {
        fail_(UpdateError::FW_VERSION_MISMATCH, "fw version mismatch after update");
        return false;
      }
      enter_(UpdateStage::DONE);
      return false;
    }
  }
  return false;
}

}  // namespace faderbuddy
