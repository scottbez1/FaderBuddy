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

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/optional.h"

#include "i2c_data.h"

#include <string>

namespace esphome {
namespace fader_buddy {

// Version of this ESPHome component, independent of the fader's firmware
// version. Logged at startup so a bug report identifies both halves.
#define FADER_BUDDY_COMPONENT_VERSION "0.3.0"


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

    // Called only from codegen to store initial haptic configs
    void store_initial_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);

    Trigger<uint8_t, uint8_t> *get_on_manual_move_trigger() const { return on_manual_move_; }
    Trigger<uint8_t, uint8_t> *get_on_raw_position_update_trigger() const { return on_raw_position_update_; }
    Trigger<bool, uint8_t> *get_on_touch_change_trigger() const { return on_touch_change_; }
    Trigger<uint8_t> *get_on_double_tap_trigger() const { return on_double_tap_; }

    protected:
        bool read_sensor_data_();
        void send_layer_haptic_config_(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);
        bool write_with_retry_(const uint8_t *data, size_t len, uint8_t retries = 3);

        Trigger<uint8_t, uint8_t> *on_manual_move_{new Trigger<uint8_t, uint8_t>()};
        Trigger<uint8_t, uint8_t> *on_raw_position_update_{new Trigger<uint8_t, uint8_t>()};
        Trigger<bool, uint8_t> *on_touch_change_{new Trigger<bool, uint8_t>()};
        Trigger<uint8_t> *on_double_tap_{new Trigger<uint8_t>()};

    private:
        void read_serial_number_();
        void read_firmware_version_();
        void read_motor_calibration_();

        // State variables
        uint32_t last_state_{0};
        // Faders normally come up idle, so this doesn't log a phantom
        // transition at startup - but a fader that boots into MODE_ERROR does
        // get reported, which is what we want.
        Mode last_mode_{MODE_INPUT_IDLE};
        std::string serial_number_;
        text_sensor::TextSensor *serial_text_sensor_{nullptr};
        HighFrequencyLoopRequester high_freq_;
        bool invert_{false};
        bool last_touch_{false};
        uint8_t last_double_tap_nonce_{0};
        uint16_t firmware_version_{FW_VERSION_NONE};
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

}  // namespace fader_buddy
}  // namespace esphome