#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"
#include "esphome/core/optional.h"

namespace esphome {
namespace motorFaderESPHomeComponent {

// Protocol v5: Layer management is now handled in firmware
// ESPHome component is a simple protocol wrapper

class MotorFaderESPHomeComponent : public PollingComponent, public i2c::I2CDevice {

 public:
     // Constructor
     MotorFaderESPHomeComponent();

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

    Trigger<uint8_t, uint8_t> *get_on_manual_move_trigger() const { return on_manual_move_; }
    Trigger<bool, uint8_t> *get_on_touch_change_trigger() const { return on_touch_change_; }
    Trigger<uint8_t> *get_on_double_tap_trigger() const { return on_double_tap_; }

    protected:
        bool read_sensor_data_();
        void send_layer_haptic_config_(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);
        bool write_with_retry_(const uint8_t *data, size_t len, uint8_t retries = 3);

        Trigger<uint8_t, uint8_t> *on_manual_move_{new Trigger<uint8_t, uint8_t>()};
        Trigger<bool, uint8_t> *on_touch_change_{new Trigger<bool, uint8_t>()};
        Trigger<uint8_t> *on_double_tap_{new Trigger<uint8_t>()};

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
};

// Action classes for automation
template<typename... Ts> class SetActiveLayerAction : public Action<Ts...> {
 public:
  SetActiveLayerAction(MotorFaderESPHomeComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, layer)

  void play(Ts... x) override { this->parent_->set_active_layer(this->layer_.value(x...)); }

 protected:
  MotorFaderESPHomeComponent *parent_;
};

template<typename... Ts> class RemoteMoveToAction : public Action<Ts...> {
 public:
  RemoteMoveToAction(MotorFaderESPHomeComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, position)
  TEMPLATABLE_VALUE(uint8_t, layer)

  void play(Ts... x) override {
    this->parent_->remote_move_to(this->position_.value(x...), this->layer_.value(x...));
  }

 protected:
  MotorFaderESPHomeComponent *parent_;
};

template<typename... Ts> class SetLayerHapticConfigAction : public Action<Ts...> {
 public:
  SetLayerHapticConfigAction(MotorFaderESPHomeComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint8_t, layer)
  TEMPLATABLE_VALUE(uint8_t, mode)
  TEMPLATABLE_VALUE(uint8_t, detent_count)
  TEMPLATABLE_VALUE(uint8_t, detent_strength)

  void play(Ts... x) override {
    this->parent_->set_layer_haptic_config(
        this->layer_.value(x...),
        this->mode_.value(x...),
        this->detent_count_.value(x...),
        this->detent_strength_.value(x...)
    );
  }

 protected:
  MotorFaderESPHomeComponent *parent_;
};

template<typename... Ts> class RunSelfCalibrationAction : public Action<Ts...> {
 public:
  RunSelfCalibrationAction(MotorFaderESPHomeComponent *parent) : parent_(parent) {}

  void play(Ts... x) override { this->parent_->run_self_calibration(); }

 protected:
  MotorFaderESPHomeComponent *parent_;
};

}  // namespace motorFaderESPHomeComponent
}  // namespace esphome