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

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/core/optional.h"

#include "i2c_data.h"

#include <string>

namespace esphome {
namespace fader_buddy {

// Version of this ESPHome component, independent of the fader's firmware
// version. Logged at startup so a bug report identifies both halves.
#define FADER_BUDDY_COMPONENT_VERSION "0.3.1"


// Protocol v5: Layer management is now handled in firmware
// ESPHome component is a simple protocol wrapper

class FaderBuddy : public PollingComponent, public i2c::I2CDevice {

 public:
     // Constructor
     FaderBuddy();

     // Standard component functions to override
     void setup() override;
     void update() override;
     void loop() override;
     void dump_config() override;
     float get_setup_priority() const override;

    // Layer management (Protocol v5: forwards to firmware)
    void set_active_layer(uint8_t layer_index);
    uint8_t get_active_layer() const;
    // Moves at the layer's configured default speed (see set_layer_default_speed).
    void remote_move_to(uint8_t position, uint8_t layer = 0);
    // speed: unitless 0-255, where LAYER_SPEED_FULL (255) is full speed and 0
    // is the slowest the fader moves smoothly. It caps speed rather than
    // scheduling the move, so a shorter move takes proportionally less time.
    void remote_move_to(uint8_t position, uint8_t layer, uint8_t speed);
    uint8_t get_position(uint8_t layer = 0) const;
    void set_layer_haptic_config(
        uint8_t layer,
        uint8_t mode,
        uint8_t detent_count = 0,
        uint8_t detent_strength = 0
    );

    void run_self_calibration();
    void set_invert(bool invert) { invert_ = invert; }
    void set_layer_value_change_min_interval(uint8_t layer, uint32_t min_interval_ms);
    // Speed used for moves on this layer when the caller doesn't name one.
    // Tracked host-side and sent with each move, rather than pushed to the
    // fader as configuration, so a layer's default costs no extra I2C traffic
    // and needs no round trip to change.
    void set_layer_default_speed(uint8_t layer, uint8_t speed);
    uint8_t get_layer_default_speed(uint8_t layer) const;

    // Firmware version reported by the fader, packed as (major << 8) | minor.
    // FW_VERSION_NONE for firmware old enough to have no version register.
    uint16_t get_firmware_version() const { return firmware_version_; }

    // Chip serial number (10-byte factory ID, read once at setup). Returns an
    // uppercase hex string (e.g. "AABBCCDDEEFF00112233"), or "" if not yet read.
    std::string get_serial_number() const { return serial_number_; }
    void set_serial_text_sensor(text_sensor::TextSensor *s) { serial_text_sensor_ = s; }
    void set_firmware_text_sensor(text_sensor::TextSensor *s) { firmware_text_sensor_ = s; }

    // Called only from codegen to store initial haptic configs
    void store_initial_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);

    // Firmware update over the I2C bootloader -- see ABOUT_I2C_BOOTLOADER.md.
    // Called only from codegen to configure the packaged application image (embedded
    // once and shared across MULTI_CONF instances; image/length/crc16 describe the
    // exact page-aligned APPCODE bytes, fw_version is read from its last 2 bytes --
    // see BL_APP_META_ADDR in bootloader_protocol.h).
    void set_firmware_image(const uint8_t *image, uint32_t length, uint16_t image_crc16, uint16_t fw_version);
    // Manual update action: starts an update and returns immediately. The transfer
    // then runs a slice at a time from loop(), so the device stays responsive and
    // coarse progress reaches the firmware version text sensor as it goes. The
    // outcome arrives on the on_firmware_update_result trigger, not from here.
    // Safe to call whether or not the fader is already at the packaged version
    // (no-ops if so). Never triggered automatically -- only when this is called.
    void update_firmware();
    // True between update_firmware() and the result trigger.
    bool firmware_update_in_progress() const { return update_stage_ != UPDATE_IDLE; }
    // Whether an update would actually do anything: an image is configured, the
    // fader isn't already running it, and there is a route to the bootloader.
    // Decided from state cached at setup, so it costs no bus traffic and can be
    // polled from a lambda; update_firmware() re-checks over the wire before
    // committing, so this is a cheap pre-filter, not the authority.
    bool firmware_update_available() const;
    // What the firmware update button presses. Same thing as update_firmware(),
    // except a press with nothing to install is rejected outright rather than
    // taking the bus to find that out.
    void request_firmware_update();

    Trigger<uint8_t, uint8_t> *get_on_manual_move_trigger() const { return on_manual_move_; }
    Trigger<uint8_t, uint8_t> *get_on_raw_position_update_trigger() const { return on_raw_position_update_; }
    Trigger<bool, uint8_t> *get_on_touch_change_trigger() const { return on_touch_change_; }
    Trigger<uint8_t> *get_on_double_tap_trigger() const { return on_double_tap_; }
    Trigger<bool, std::string> *get_on_firmware_update_result_trigger() const { return on_firmware_update_result_; }

    protected:
        bool read_sensor_data_();
        void send_layer_haptic_config_(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);
        bool write_with_retry_(const uint8_t *data, size_t len, uint8_t retries = 3);

        Trigger<uint8_t, uint8_t> *on_manual_move_{new Trigger<uint8_t, uint8_t>()};
        Trigger<uint8_t, uint8_t> *on_raw_position_update_{new Trigger<uint8_t, uint8_t>()};
        Trigger<bool, uint8_t> *on_touch_change_{new Trigger<bool, uint8_t>()};
        Trigger<uint8_t> *on_double_tap_{new Trigger<uint8_t>()};
        Trigger<bool, std::string> *on_firmware_update_result_{new Trigger<bool, std::string>()};

    private:
        // Probe REG_VERSION and, if the answer is usable, finish initializing.
        // first_attempt distinguishes the call from setup() (retries hard, and
        // decides whether to fail the component) from the periodic retry driven
        // by update() while awaiting_device_ is set.
        void probe_and_init_(bool first_attempt);
        // Drives the bounded re-probe backoff while awaiting_device_ is set.
        void retry_probe_();
        void read_serial_number_();
        void read_firmware_version_();
        // How the firmware version text sensor renders the fader's current
        // state: "1.5", "1.0 or older", or "bootloader (no app)".
        std::string firmware_version_text_() const;
        // Push an arbitrary string to that sensor. Used to report update
        // status there, since a fader mid-update has no version to report.
        void publish_firmware_text_(const std::string &text);
        void read_motor_calibration_();

        // State variables
        uint32_t last_state_{0};
        // Faders normally come up idle, so this doesn't log a phantom
        // transition at startup - but a fader that boots into MODE_ERROR does
        // get reported, which is what we want.
        Mode last_mode_{MODE_INPUT_IDLE};
        std::string serial_number_;
        text_sensor::TextSensor *serial_text_sensor_{nullptr};
        text_sensor::TextSensor *firmware_text_sensor_{nullptr};
        HighFrequencyLoopRequester high_freq_;
        bool invert_{false};
        bool last_touch_{false};
        uint8_t last_double_tap_nonce_{0};
        uint16_t firmware_version_{FW_VERSION_NONE};
        // Set when the version probe at setup saw the bootloader's marker rather
        // than a protocol version, i.e. the fader has no working app image. The
        // one case where an update is viable despite no readable app version.
        bool bootloader_resident_{false};
        // The fader never answered the setup version probe, but a firmware_image
        // is configured, so the component stays alive rather than being failed:
        // update() keeps re-probing, and the firmware update button stays
        // available to recover a fader that is wedged rather than absent.
        // mark_failed() would foreclose both - ESPHome calls neither loop() nor
        // update() on a failed component.
        bool awaiting_device_{false};
        // Bounded, doubling backoff for those re-probes; once it runs out the
        // fader is left alone rather than tying up the bus on every poll.
        uint32_t retry_probe_at_{0};
        uint32_t retry_probe_backoff_ms_{0};
        uint8_t retry_probes_left_{0};
        bool speed_supported_{false};
        bool warned_speed_unsupported_{false};  // warn once, not once per move

        // Per-layer state for value change rate limiting and position tracking
        struct LayerState {
            uint32_t value_change_min_interval{0};  // 0 = no rate limiting
            uint32_t last_trigger_time{0};
            uint8_t deferred_value{0};  // USER-FACING position for deferred trigger
            bool has_deferred_value{false};
            uint16_t last_hw_position{0};  // Last HARDWARE position from firmware (0-255, raw from I2C)
            uint8_t last_position_nonce{0};
            uint8_t default_speed{LAYER_SPEED_FULL};  // speed for moves that don't name one
        };
        LayerState layer_states_[8] = {};

        // Initial haptic configurations (set during codegen, sent once during setup)
        struct InitialHapticConfig {
            uint8_t layer;
            uint8_t mode;
            uint8_t detent_count;
            uint8_t detent_strength;
            bool valid;
        };
        InitialHapticConfig initial_haptic_configs_[8] = {};

        // --- Firmware update state (ABOUT_I2C_BOOTLOADER.md) ---
        const uint8_t *firmware_image_{nullptr};
        uint32_t firmware_image_length_{0};
        uint16_t firmware_image_crc16_{0};
        uint16_t firmware_fw_version_{0};

        // Guards against two faders updating the shared I2C bus at once. Held for
        // the whole multi-tick sequence, so unlike the per-instance update_stage_
        // this also stops a second fader starting one while the first is mid-flight.
        static bool s_update_in_progress;

        // --- Update state machine (driven by update_tick_() from loop()) ---
        // The transfer is sliced across loop iterations rather than run inline:
        // it takes seconds end to end, and blocking that long starves the API,
        // WiFi and every other component - and makes progress reporting
        // impossible, since queued entity states only go out from the loop.
        enum UpdateStage : uint8_t {
            UPDATE_IDLE = 0,
            UPDATE_WAIT_TOUCH_IDLE,  // bounded wait for the user to let go
            UPDATE_PROBE,            // app or bootloader? and the go/no-go checks
            UPDATE_WAIT_MARKER,      // after REG_ENTER_BOOTLOADER
            UPDATE_ERASE,            // status check, then issue the erase
            UPDATE_ERASE_WAIT,       // target is stalled in NVM, poll it back
            UPDATE_WRITE,            // stream pages, budgeted per tick
            UPDATE_WRITE_CHECK,      // post-write NVM status, then ask for the CRC
            UPDATE_CRC_WAIT,         // poll for the whole-image CRC
            UPDATE_WAIT_APP,         // after RUN_APP
            UPDATE_CONFIRM,          // REG_FW_VERSION matches the packaged image
        };
        // How long one tick may spend streaming pages before yielding. Long
        // enough to get several 64-byte pages out per loop iteration, short
        // enough that nothing else notices.
        static constexpr uint32_t UPDATE_WRITE_BUDGET_MS = 20;

        UpdateStage update_stage_{UPDATE_IDLE};
        uint32_t update_deadline_{0};   // millis deadline for the polling stages
        uint32_t update_page_{0};       // next page to write
        uint8_t update_progress_pct_{0xFF};  // last published step; 0xFF = none yet

        void update_tick_();
        void enter_update_stage_(UpdateStage stage, uint32_t timeout_ms = 0);
        void publish_update_progress_(const char *label, uint8_t pct);
        void log_update_starting_();
        void finish_update_(bool ok, const std::string &error);

        uint8_t get_last_mode_() const;

        // Bootloader wire-protocol primitives -- port of the jig's reference
        // implementation, see production_tools/programAndTest/src/fader_buddy_bootloader.cpp.
        bool bl_write_retry_(const uint8_t *data, size_t len, uint8_t attempts = 3, uint32_t retry_delay_ms = 5);
        bool bl_read_retry_(const uint8_t *reg, size_t reg_len, uint8_t *out, size_t out_len,
                            uint8_t attempts = 3, uint32_t retry_delay_ms = 5);
        bool bl_read_bare_retry_(uint8_t *out, size_t len, uint8_t attempts, uint32_t retry_delay_ms);
        bool bl_read_version_byte_(uint8_t &version);
        bool bl_enter_bootloader_();
        bool bl_get_status_(uint8_t &bl_version, uint8_t &status, uint8_t &last_error);
        // Erase and whole-image CRC are each split into "ask" and "poll for the
        // answer": both stall the target long enough that waiting inline would
        // be the one thing still blocking the loop. See UPDATE_ERASE_WAIT and
        // UPDATE_CRC_WAIT in update_tick_().
        bool bl_request_erase_app_();
        bool bl_set_page_addr_(uint16_t addr);
        bool bl_send_frame_(const uint8_t *data16);
        bool bl_request_image_crc16_(uint16_t addr, uint16_t len);
        bool bl_poll_image_crc16_(uint16_t &crc);
        bool bl_run_app_();
        bool read_fw_version_(uint16_t &version);
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

// A press runs the fader_buddy.update_firmware action, but only when there is
// actually something to install -- see firmware_update_available().
// Writing flash takes tens of seconds and the fader is unusable meanwhile, so
// this is entity_category "config" as well.
class FirmwareUpdateButton : public button::Button, public Parented<FaderBuddy> {
 protected:
  void press_action() override { this->parent_->request_firmware_update(); }
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