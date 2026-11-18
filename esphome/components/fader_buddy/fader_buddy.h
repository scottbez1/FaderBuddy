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

#include <stdarg.h>

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/optional.h"

#include "fader_buddy_update.h"
#include "i2c_data.h"

// The host library (hostLib/ at the repo root). These files are copies -
// edit hostLib/src/ and run ci/util/check_i2c_data_sync.py --fix.
#include "fader_buddy_host.h"

namespace esphome {
namespace fader_buddy {

// Version of this ESPHome component, independent of the fader's firmware
// version. Logged at startup so a bug report identifies both halves.
#define FADER_BUDDY_COMPONENT_VERSION "0.7.0"

// The host library's Transport and Platform, over ESPHome's I2C device and HAL.
class EsphomeTransport : public ::faderbuddy::Transport {
 public:
  explicit EsphomeTransport(i2c::I2CDevice *device) : device_(device) {}
  bool write(const uint8_t *data, size_t len) override {
    return device_->write(data, len) == i2c::ErrorCode::NO_ERROR;
  }
  bool read(uint8_t *data, size_t len) override { return device_->read(data, len) == i2c::ErrorCode::NO_ERROR; }
  bool write_read(const uint8_t *write_data, size_t write_len, uint8_t *read_data, size_t read_len) override {
    return device_->write_read(write_data, write_len, read_data, read_len) == i2c::ErrorCode::NO_ERROR;
  }

 private:
  i2c::I2CDevice *device_;
};

class EsphomePlatform : public ::faderbuddy::Platform {
 public:
  uint32_t millis() override { return esphome::millis(); }
  void delay_ms(uint32_t ms) override { esphome::delay(ms); }
  void log(::faderbuddy::LogLevel level, const char *format, va_list args) override;
};

// Constructed before the library base, which holds references to both.
struct FaderBuddyHostAdapters {
  explicit FaderBuddyHostAdapters(i2c::I2CDevice *device) : host_transport_(device) {}
  EsphomeTransport host_transport_;
  EsphomePlatform host_platform_;
};

// The fader itself is ::faderbuddy::FaderBuddy; this adds what is specific to
// ESPHome: the component lifecycle, automation triggers, Home Assistant
// entities, and serializing firmware updates across faders sharing a bus.
class FaderBuddy : public PollingComponent,
                   public i2c::I2CDevice,
                   private FaderBuddyHostAdapters,
                   public ::faderbuddy::FaderBuddy,
                   protected ::faderbuddy::FaderBuddyListener {
 public:
  FaderBuddy();

  // Standard component functions to override
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // The rest of the public interface - layers, moves, haptics, calibration,
  // firmware version and updates - is inherited from ::faderbuddy::FaderBuddy.
  // See hostLib/src/fader_buddy_host.h.

  // Chip serial number (10-byte factory ID, read once at setup). Returns an
  // uppercase hex string (e.g. "AABBCCDDEEFF00112233"), or "" if not yet read.
  std::string get_serial_number() const { return ::faderbuddy::FaderBuddy::get_serial_number(); }
  void set_serial_text_sensor(text_sensor::TextSensor *s) { serial_text_sensor_ = s; }
  // Diagnostic text sensor: firmware version, "not responding", a required
  // UPDI reflash, or update progress.
  void set_status_text_sensor(text_sensor::TextSensor *s) { status_text_sensor_ = s; }
#ifdef USE_UPDATE
  void set_update_entity(FaderBuddyUpdate *u) { update_entity_ = u; }
#endif

  // Manual update action: starts an update and returns immediately. The
  // transfer then runs a slice at a time from loop(). The outcome arrives on
  // the on_firmware_update_result trigger, not from here. Only one fader
  // updates at a time, since they share the bus: a request made while another
  // fader is updating waits, shown as "Update pending", and starts when the
  // bus frees up.
  void update_firmware() override;
  // True between update_firmware() and the result trigger, waiting time included.
  bool firmware_update_in_progress() const override {
    return ::faderbuddy::FaderBuddy::firmware_update_in_progress() || update_pending_;
  }

  Trigger<uint8_t, uint8_t> *get_on_manual_move_trigger() const { return on_manual_move_; }
  Trigger<uint8_t, uint8_t> *get_on_raw_position_update_trigger() const { return on_raw_position_update_; }
  Trigger<bool, uint8_t> *get_on_touch_change_trigger() const { return on_touch_change_; }
  Trigger<uint8_t> *get_on_double_tap_trigger() const { return on_double_tap_; }
  Trigger<bool, std::string> *get_on_firmware_update_result_trigger() const { return on_firmware_update_result_; }

 protected:
  // FaderBuddyListener
  void on_manual_move(uint8_t position, uint8_t layer) override { on_manual_move_->trigger(position, layer); }
  void on_raw_position_update(uint8_t position, uint8_t layer) override {
    on_raw_position_update_->trigger(position, layer);
  }
  void on_touch_change(bool touched, uint8_t layer) override { on_touch_change_->trigger(touched, layer); }
  void on_double_tap(uint8_t layer) override { on_double_tap_->trigger(layer); }
  void on_serial_number(const char *serial) override;
  void on_status_changed() override;
  void on_firmware_update_stage(const char *label) override;
  void on_firmware_update_progress(const char *label, uint8_t percent) override;
  void on_firmware_update_result(bool success, const char *error) override {
    on_firmware_update_result_->trigger(success, std::string(error));
  }
  // ::faderbuddy::FaderBuddy
  void on_update_attempt_finished_(bool success, const char *error) override;

  Trigger<uint8_t, uint8_t> *on_manual_move_{new Trigger<uint8_t, uint8_t>()};
  Trigger<uint8_t, uint8_t> *on_raw_position_update_{new Trigger<uint8_t, uint8_t>()};
  Trigger<bool, uint8_t> *on_touch_change_{new Trigger<bool, uint8_t>()};
  Trigger<uint8_t> *on_double_tap_{new Trigger<uint8_t>()};
  Trigger<bool, std::string> *on_firmware_update_result_{new Trigger<bool, std::string>()};

 private:
  // Claim the bus and start the transfer. The caller has already checked that
  // the bus is free.
  void start_update_();
  // The fader's version as reported to the update entity. Home Assistant
  // compares it against the packaged version, so it must parse as a version.
  // Extra detail goes in a semver build-metadata suffix, which doesn't affect
  // the comparison:
  //   "1.4"                 - running 1.4
  //   "1.1+updi-required"   - running 1.1, too old to enter its bootloader
  //                           over I2C (see FW_VERSION_BOOTLOADER_ENTRY)
  //   "1.0+updi-required"   - no version register, so 1.0 or older
  //   "0.0+bootloader"      - in its bootloader, no app installed
  //   "0.0+unreachable"     - never answered a probe
  std::string firmware_version_string_() const;
  // Human-readable text for the status sensor: the firmware status, plus the
  // calibration status when there is something to say about it.
  std::string status_text_() const;
  std::string firmware_status_text_() const;
  // Publish arbitrary text to the status sensor, for transient states that
  // status_text_() doesn't cover (update progress, failure reasons).
  void publish_status_(const std::string &text);
  // Publish status_text_() and the update entity state together.
  void publish_firmware_state_();

  text_sensor::TextSensor *serial_text_sensor_{nullptr};
  text_sensor::TextSensor *status_text_sensor_{nullptr};
#ifdef USE_UPDATE
  FaderBuddyUpdate *update_entity_{nullptr};
#endif
  HighFrequencyLoopRequester high_freq_;

  // Guards against two faders updating the shared I2C bus at once. Held for
  // the whole multi-tick sequence, so it also stops a second fader starting
  // one while the first is mid-flight.
  static bool s_update_in_progress;
  // This fader holds s_update_in_progress.
  bool holds_bus_{false};
  // An update was requested while another fader held the bus. loop() starts
  // it once s_update_in_progress clears, so waiting faders go in loop (YAML)
  // order rather than request order.
  bool update_pending_{false};
};

// Action classes for automation
// A press runs the same thing as the fader_buddy.run_self_calibration action.
// The fader sweeps to both ends and characterises its motor, which takes a few
// seconds and moves the carriage - hence entity_category "config", so Home
// Assistant files it with the device's settings rather than its controls.
class SelfCalibrationButton : public button::Button, public Parented<FaderBuddy> {
 protected:
  void press_action() override { this->parent_->run_self_calibration(); }
};

template<typename... Ts> class SetActiveLayerAction : public Action<Ts...> {
 public:
  SetActiveLayerAction(FaderBuddy *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, layer)

  void play(const Ts &...x) override { this->parent_->set_active_layer(this->layer_.value(x...)); }

 protected:
  FaderBuddy *parent_;
};

template<typename... Ts> class RemoteMoveToAction : public Action<Ts...> {
 public:
  RemoteMoveToAction(FaderBuddy *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, position)
  TEMPLATABLE_VALUE(uint8_t, layer)
  TEMPLATABLE_VALUE(uint8_t, speed)

  void play(const Ts &...x) override {
    uint8_t position = this->position_.value(x...);
    uint8_t layer = this->layer_.value(x...);
    // An unset speed means "whatever this layer is configured to use", which
    // is not the same as any particular value the caller could pass.
    if (this->speed_.has_value()) {
      this->parent_->remote_move_to(position, layer, this->speed_.value(x...));
    } else {
      this->parent_->remote_move_to(position, layer);
    }
  }

 protected:
  FaderBuddy *parent_;
};

template<typename... Ts> class SetLayerHapticConfigAction : public Action<Ts...> {
 public:
  SetLayerHapticConfigAction(FaderBuddy *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, layer)
  TEMPLATABLE_VALUE(uint8_t, mode)
  TEMPLATABLE_VALUE(uint8_t, detent_count)
  TEMPLATABLE_VALUE(uint8_t, detent_strength)

  void play(const Ts &...x) override {
    this->parent_->set_layer_haptic_config(
        this->layer_.value(x...),
        this->mode_.value(x...),
        this->detent_count_.value(x...),
        this->detent_strength_.value(x...)
    );
  }

 protected:
  FaderBuddy *parent_;
};

template<typename... Ts> class RunSelfCalibrationAction : public Action<Ts...> {
 public:
  RunSelfCalibrationAction(FaderBuddy *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->run_self_calibration(); }

 protected:
  FaderBuddy *parent_;
};

template<typename... Ts> class UpdateFirmwareAction : public Action<Ts...> {
 public:
  UpdateFirmwareAction(FaderBuddy *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->update_firmware(); }

 protected:
  FaderBuddy *parent_;
};

}  // namespace fader_buddy
}  // namespace esphome
