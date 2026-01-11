#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"
#include "esphome/core/optional.h"

namespace esphome {
namespace motorFaderESPHomeComponent {

// Per-layer state structure
struct LayerState {
  // Position to restore when switching to this layer (USER-FACING position 0-255)
  // This is the position the user expects to see, accounting for invert_ setting
  uint8_t restore_position = 0;

  // The haptic config nonce value for this layer (maps to layer index)
  uint8_t expected_nonce = 0;

  // Cached haptic configuration for this layer (32-bit packed value)
  uint32_t haptic_config = 0;

  // Last confirmed position from firmware when this layer was active (HARDWARE position 0-255)
  // This is the raw position from firmware, not accounting for invert_ setting
  uint8_t last_confirmed_hw_position = 0;

  // Last known position nonce when this layer was active
  uint8_t last_position_nonce = 0;

  // Track if layer has ever been active
  bool has_been_initialized = false;
};

class MotorFaderESPHomeComponent : public PollingComponent, public i2c::I2CDevice {

 public:
     // Constructor
     MotorFaderESPHomeComponent();

     // Standard component functions to override
     void setup() override;
     void update() override;
     void dump_config() override;
     float get_setup_priority() const override;

    // Layer management
    void set_active_layer(uint8_t layer_index);
    uint8_t get_active_layer() const { return active_layer_; }
    void remote_move_to(uint8_t position, uint8_t layer = 0);
    uint8_t get_position(uint8_t layer = 0) const;
    void set_layer_haptic_config(
        uint8_t layer,
        uint8_t mode,
        uint8_t detent_count = 0,
        uint8_t detent_strength = 0,
        uint8_t target_position = 0
    );

    void run_self_calibration();
    void set_invert(bool invert) { invert_ = invert; }
    void set_value_change_rate_limit(uint32_t rate_limit_ms) { value_change_rate_limit_ = rate_limit_ms; }

    Trigger<uint8_t, uint8_t> *get_on_manual_move_trigger() const { return on_manual_move_; }
    Trigger<bool, uint8_t> *get_on_touch_change_trigger() const { return on_touch_change_; }
    Trigger<uint8_t> *get_on_double_tap_trigger() const { return on_double_tap_; }

    protected:
        bool read_sensor_data_();

        Trigger<uint8_t, uint8_t> *on_manual_move_{new Trigger<uint8_t, uint8_t>()};
        Trigger<bool, uint8_t> *on_touch_change_{new Trigger<bool, uint8_t>()};
        Trigger<uint8_t> *on_double_tap_{new Trigger<uint8_t>()};

    private:
        // Helper functions
        uint32_t make_haptic_config_internal_(
            uint8_t nonce,
            uint8_t mode,
            uint8_t detent_count,
            uint8_t detent_strength,
            uint8_t target_position
        );
        void write_haptic_config_(uint32_t config);
        void write_target_position_(uint8_t position);

        // Layer management
        uint8_t active_layer_{0};
        LayerState layer_state_[8];
        optional<uint8_t> pending_layer_switch_;
        uint32_t layer_switch_timeout_{0};
        static constexpr uint32_t LAYER_SWITCH_TIMEOUT_MS = 500;

        // Existing state variables
        uint16_t last_hw_position_{0};  // Last HARDWARE position from firmware (0-255, raw from I2C)
        uint8_t last_position_nonce_{0};
        uint32_t last_state_{0};
        HighFrequencyLoopRequester high_freq_;
        bool invert_{false};
        uint32_t value_change_rate_limit_{0};  // 0 = no rate limiting
        uint32_t last_trigger_time_{0};
        uint8_t deferred_value_{0};  // USER-FACING position for deferred trigger
        uint8_t deferred_value_layer_{0};
        bool has_deferred_value_{false};
        bool last_touch_{false};
        uint8_t last_double_tap_nonce_{0};
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
        this->detent_strength_.value(x...),
        0  // target_position
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