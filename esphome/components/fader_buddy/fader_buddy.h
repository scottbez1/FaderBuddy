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
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"
#include "esphome/core/optional.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace fader_buddy {

// Protocol v5: Layer management is now handled in firmware
// ESPHome component is a simple protocol wrapper

class FaderBuddy : public PollingComponent, public i2c::I2CDevice {

 public:
     // Constructor
     FaderBuddy();

     // Standard component functions to override
     void setup() override;
     void update() override;
     void dump_config() override;
     float get_setup_priority() const override;

    // Layer management (Protocol v5: forwards to firmware)
    void set_active_layer(uint8_t layer_index);
    uint8_t get_active_layer() const;
    void remote_move_to(uint8_t position, uint8_t layer = 0);
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

    // Called only from codegen to store initial haptic configs
    void store_initial_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);

    // Firmware update over the I2C bootloader -- see ABOUT_I2C_BOOTLOADER.md section 11.
    // Called only from codegen to configure the packaged application image (embedded
    // once and shared across MULTI_CONF instances; image/length/crc16 describe the
    // exact page-aligned APPCODE bytes, fw_version is read from its last 2 bytes --
    // see BL_APP_META_ADDR in bootloader_protocol.h).
    void set_firmware_image(const uint8_t *image, uint32_t length, uint16_t image_crc16, uint16_t fw_version);
    void set_max_update_attempts(uint8_t max_attempts) { max_update_attempts_ = max_attempts; }
    // Manual update action: blocks until the update finishes or fails. Safe to call
    // whether or not the fader is already at the packaged version (no-ops if so).
    // Never triggered automatically -- only ever runs when this is called.
    void update_firmware();

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
        // State variables
        uint32_t last_state_{0};
        HighFrequencyLoopRequester high_freq_;
        bool invert_{false};
        bool last_touch_{false};
        uint8_t last_double_tap_nonce_{0};

        // Per-layer state for value change rate limiting and position tracking
        struct LayerState {
            uint32_t value_change_min_interval{0};  // 0 = no rate limiting
            uint32_t last_trigger_time{0};
            uint8_t deferred_value{0};  // USER-FACING position for deferred trigger
            bool has_deferred_value{false};
            uint16_t last_hw_position{0};  // Last HARDWARE position from firmware (0-255, raw from I2C)
            uint8_t last_position_nonce{0};
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

        // --- Firmware update state (ABOUT_I2C_BOOTLOADER.md section 11) ---
        const uint8_t *firmware_image_{nullptr};
        uint32_t firmware_image_length_{0};
        uint16_t firmware_image_crc16_{0};
        uint16_t firmware_fw_version_{0};
        uint8_t max_update_attempts_{3};
        ESPPreferenceObject update_attempts_pref_;

        // Guards against two faders updating the shared I2C bus at once; a single
        // update() call blocks for the whole sequence, so this only matters across
        // instances (e.g. concurrent HA service calls landing on different tasks).
        static bool s_update_in_progress;

        uint8_t get_last_mode_() const;

        bool perform_firmware_update_(std::string &error_out);

        // Bootloader wire-protocol primitives -- port of the jig's reference
        // implementation, see production_tools/programAndTest/src/fader_buddy_bootloader.cpp.
        bool bl_write_retry_(const uint8_t *data, size_t len, uint8_t attempts = 3, uint32_t retry_delay_ms = 5);
        bool bl_read_retry_(const uint8_t *reg, size_t reg_len, uint8_t *out, size_t out_len,
                            uint8_t attempts = 3, uint32_t retry_delay_ms = 5);
        bool bl_read_bare_retry_(uint8_t *out, size_t len, uint8_t attempts, uint32_t retry_delay_ms);
        bool bl_read_version_byte_(uint8_t &version);
        bool bl_enter_bootloader_();
        bool bl_wait_for_marker_(uint32_t timeout_ms);
        bool bl_wait_for_app_(uint32_t timeout_ms, uint8_t &version);
        bool bl_get_status_(uint8_t &bl_version, uint8_t &status, uint8_t &last_error);
        bool bl_erase_app_();
        bool bl_set_page_addr_(uint16_t addr);
        bool bl_send_frame_(const uint8_t *data16);
        bool bl_get_image_crc16_(uint16_t addr, uint16_t len, uint16_t &crc);
        bool bl_run_app_();
        bool read_fw_version_(uint16_t &version);
};

// Action classes for automation
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

  void play(const Ts &...x) override {
    this->parent_->remote_move_to(this->position_.value(x...), this->layer_.value(x...));
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