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

#include "fader_buddy.h"

#include <cstdio>

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace fader_buddy {

static const char *const TAG = "fader_buddy";

bool FaderBuddy::s_update_in_progress = false;

// The library's levels are ESPHome's, offset by one (ESPHome's 0 is NONE).
void EsphomePlatform::log(::faderbuddy::LogLevel level, const char *format, va_list args) {
  int esphome_level = (int) level + ESPHOME_LOG_LEVEL_ERROR;
  if (esphome_level > ESPHOME_LOG_LEVEL) {
    return;
  }
  esp_log_vprintf_(esphome_level, TAG, __LINE__, format, args);
}

FaderBuddy::FaderBuddy()
    : PollingComponent(),
      i2c::I2CDevice(),
      FaderBuddyHostAdapters(this),
      ::faderbuddy::FaderBuddy(host_transport_, host_platform_) {
  this->set_listener(this);
}

void FaderBuddy::setup() {
  ESP_LOGCONFIG(TAG, "Setting up FaderBuddy (component %s) at 0x%02X...", FADER_BUDDY_COMPONENT_VERSION,
                this->get_i2c_address());

#ifdef USE_UPDATE
  // Take the latest version from the packaged image's FW_VERSION footer rather
  // than from codegen, so there's only one source for it.
  if (this->update_entity_ != nullptr && this->has_firmware_image()) {
    char buf[16];
    uint16_t v = this->get_packaged_firmware_version();
    snprintf(buf, sizeof(buf), "%d.%d", v >> 8, v & 0xFF);
    this->update_entity_->set_latest_version(buf);
    this->update_entity_->set_title("FaderBuddy fader firmware");
  }
#endif

  // Done before the probe, not after, so it still happens for a fader that
  // doesn't answer yet: the loop rate is a property of this config, not of the
  // fader.
  if (this->get_update_interval() < App.get_loop_interval()) {
    high_freq_.start();
  }

  // A fader that doesn't answer is not marked failed: ESPHome would stop
  // calling update() and loop(), which ends both the re-probing and any
  // firmware update that could recover a wedged fader. Only an incompatible
  // protocol, which needs a UPDI reflash, fails the component (see
  // on_status_changed()).
  this->begin();
}

// PollingComponent drives update() from the scheduler, not from here, so this
// costs nothing when no update is running.
void FaderBuddy::loop() {
  ::faderbuddy::FaderBuddy::loop();
  if (update_pending_ && !s_update_in_progress) {
    update_pending_ = false;
    start_update_();
  }
}

void FaderBuddy::update() { this->poll(); }

void FaderBuddy::dump_config() {
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication failed");
  }

  ESP_LOGCONFIG(TAG, "  Component Version: %s", FADER_BUDDY_COMPONENT_VERSION);
  ESP_LOGCONFIG(TAG, "  Host Library Version: %s", FADER_BUDDY_HOSTLIB_VERSION);
  if (this->is_awaiting_device()) {
    ESP_LOGW(TAG, "  Not responding - re-probing");
  }
  ESP_LOGCONFIG(TAG, "  Status: %s", this->status_text_().c_str());
  if (this->has_firmware_image()) {
    uint16_t v = this->get_packaged_firmware_version();
    ESP_LOGCONFIG(TAG, "  Packaged Firmware: %d.%d (%u bytes)", v >> 8, v & 0xFF,
                  (unsigned) this->get_packaged_firmware_length());
  }

  std::string serial = this->get_serial_number();
  if (!serial.empty()) {
    ESP_LOGCONFIG(TAG, "  Serial Number: %s", serial.c_str());
  }
  LOG_TEXT_SENSOR("  ", "Serial Number", this->serial_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Status", this->status_text_sensor_);

  LOG_UPDATE_INTERVAL(this);
}

float FaderBuddy::get_setup_priority() const { return setup_priority::DATA; }

// --- Library events ---

void FaderBuddy::on_serial_number(const char *serial) {
  if (this->serial_text_sensor_ != nullptr) {
    this->serial_text_sensor_->publish_state(serial);
  }
}

void FaderBuddy::on_status_changed() {
  this->publish_firmware_state_();
  // ESPHome never calls loop() or update() on a failed component, which is
  // right for a fader whose protocol is too old to talk to at all.
  if (this->is_incompatible() && !this->is_failed()) {
    this->mark_failed();
  }
}

// Report an update stage that has no percentage yet. The status sensor gets the
// label, and the update entity switches to installing right away rather than
// waiting for the first page to be written.
void FaderBuddy::on_firmware_update_stage(const char *label) {
  publish_status_(std::string("Updating: ") + label);
#ifdef USE_UPDATE
  if (this->update_entity_ != nullptr) {
    this->update_entity_->publish_installing();
  }
#endif
}

// Coarse progress, already thinned to 5% steps by the library.
//
// Whether Home Assistant sees each step depends on `api: batch_delay:`. States
// are batched per entity and deduplicated, so with the default 100ms delay HA
// sees only the steps that happen to straddle a flush. The log always shows all
// of them, and the terminal states always get through.
void FaderBuddy::on_firmware_update_progress(const char *label, uint8_t percent) {
  char buf[48];
  snprintf(buf, sizeof(buf), "Updating: %s %u%%", label, percent);
  publish_status_(buf);
#ifdef USE_UPDATE
  if (this->update_entity_ != nullptr) {
    this->update_entity_->publish_progress(percent);
  }
#endif
}

void FaderBuddy::on_update_attempt_finished_(bool success, const char *error) {
  if (holds_bus_) {
    holds_bus_ = false;
    s_update_in_progress = false;
  }
  if (!success) {
    // The library has already republished the fader's state after the
    // attempt. The update entity has no error field, so only the status sensor
    // can show why it failed.
    publish_status_(status_text_() + " - update failed: " + error);
  }
}

// --- Firmware update, serialized across faders ---

void FaderBuddy::update_firmware() {
  if (this->reject_update_request_()) {
    return;
  }
  if (update_pending_) {
    // The waiting request will report the result.
    ESP_LOGD(TAG, "update_firmware: already pending");
    return;
  }
  // Wait for the bus rather than refuse: Home Assistant's "update all" asks
  // for every fader at once, so this is the normal case. loop() starts it.
  if (s_update_in_progress) {
    ESP_LOGI(TAG, "update_firmware: another fader is updating, pending");
    update_pending_ = true;
    publish_firmware_state_();
    return;
  }
  start_update_();
}

void FaderBuddy::start_update_() {
  // Claim the bus before the first tick, so a second request landing later in
  // the same loop iteration waits rather than interleaves.
  s_update_in_progress = true;
  holds_bus_ = true;
  this->begin_update_();
}

// --- Status text and the update entity ---

std::string FaderBuddy::firmware_version_string_() const {
  if (this->is_awaiting_device()) {
    return "0.0+unreachable";
  }
  if (this->is_bootloader_resident()) {
    return "0.0+bootloader";
  }
  // Flag a fader that can't be updated over I2C here as well as in the status
  // text, since the update entity is where a user will look first.
  const char *suffix = this->bootloader_entry_supported() ? "" : "+updi-required";
  char version[8];
  // FW_VERSION_NONE formats as "1.0": no version register means 1.0 or older.
  ::faderbuddy::format_fw_version(this->get_firmware_version(), version, sizeof(version));
  return std::string(version) + suffix;
}

std::string FaderBuddy::status_text_() const {
  std::string text = firmware_status_text_();
  switch (this->get_calibration_status()) {
    case CALIBRATION_UNKNOWN:
      break;
    case CALIBRATION_NEEDED:
      text += " - motor not calibrated, run Self Calibration";
      break;
    case CALIBRATION_RUNNING:
      text += " - calibrating";
      break;
    case CALIBRATION_SUCCEEDED:
      text += " - calibration succeeded";
      break;
    case CALIBRATION_MOTOR_DEFAULTS:
      text += " - calibration incomplete: motor measurement failed, using default motor tuning";
      break;
    case CALIBRATION_FAILED:
      text += " - calibration failed: no fader travel detected, check motor and potentiometer wiring";
      break;
  }
  return text;
}

// Human-readable firmware version, for the status text sensor. Also says
// whether an update is available, or why it can't be installed over I2C.
std::string FaderBuddy::firmware_status_text_() const {
  if (this->is_awaiting_device()) {
    return "Not responding";
  }
  if (this->is_bootloader_resident()) {
    return "Bootloader - no application installed";
  }

  std::string text;
  uint16_t version = this->get_firmware_version();
  if (!::faderbuddy::fw_version_known(version)) {
    text = "Firmware 1.0 or older";
  } else {
    char buf[8];
    ::faderbuddy::format_fw_version(version, buf, sizeof(buf));
    text = std::string("Firmware ") + buf;
  }

  if (!this->has_firmware_image()) {
    return text;  // no packaged image, so no update to mention
  }
  char packaged[8];
  ::faderbuddy::format_fw_version(this->get_packaged_firmware_version(), packaged, sizeof(packaged));

  if (!this->bootloader_entry_supported()) {
    return text + " - cannot update to " + packaged + " over I2C, needs a one-time UPDI reflash";
  }
  if (this->firmware_update_offered()) {
    return text + " - update to " + packaged + " available";
  }
  return text;
}

void FaderBuddy::publish_status_(const std::string &text) {
  if (this->status_text_sensor_ != nullptr) {
    this->status_text_sensor_->publish_state(text);
  }
}

// Called whenever the fader's firmware state changes, so the status sensor and
// the update entity stay in sync.
void FaderBuddy::publish_firmware_state_() {
  // A waiting fader keeps showing as pending, whatever else changes while it
  // waits (a re-probe, a check for updates), until its turn comes.
  if (update_pending_) {
    publish_status_("Update pending");
#ifdef USE_UPDATE
    if (this->update_entity_ != nullptr) {
      this->update_entity_->publish_pending();
    }
#endif
    return;
  }
  publish_status_(status_text_());
#ifdef USE_UPDATE
  if (this->update_entity_ != nullptr) {
    this->update_entity_->publish_versions(firmware_version_string_(), this->firmware_update_offered());
  }
#endif
}

}  // namespace fader_buddy
}  // namespace esphome
