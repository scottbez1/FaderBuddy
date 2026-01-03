#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace motorFaderESPHomeComponent {

class MotorFaderESPHomeComponent : public PollingComponent, public i2c::I2CDevice {

 public:
     // Constructor
     MotorFaderESPHomeComponent() : PollingComponent(), i2c::I2CDevice() {};
    
     // Standard component functions to override
     void setup() override;
     void update() override;
     void dump_config() override;
     float get_setup_priority() const override;

    void remote_move_to(uint8_t position);
    void run_self_calibration();
    void set_invert(bool invert) { invert_ = invert; }
    void set_value_change_rate_limit(uint32_t rate_limit_ms) { value_change_rate_limit_ = rate_limit_ms; }

    Trigger<uint8_t> *get_on_manual_move_trigger() const { return on_manual_move_; }
    Trigger<bool> *get_on_touch_change_trigger() const { return on_touch_change_; }
    Trigger<> *get_on_double_tap_trigger() const { return on_double_tap_; }

    protected:
        bool read_sensor_data_();

        Trigger<uint8_t> *on_manual_move_{new Trigger<uint8_t>()};
        Trigger<bool> *on_touch_change_{new Trigger<bool>()};
        Trigger<> *on_double_tap_{new Trigger<>()};
    
    private:
        uint16_t last_position_;
        uint8_t last_position_nonce_;
        uint32_t last_state_;
        HighFrequencyLoopRequester high_freq_;
        bool invert_{false};
        uint32_t value_change_rate_limit_{0};  // 0 = no rate limiting
        uint32_t last_trigger_time_{0};
        uint8_t deferred_value_{0};
        bool has_deferred_value_{false};
        bool last_touch_{false};
        uint8_t last_double_tap_nonce_{0};
};

}  // namespace motorFaderESPHomeComponent
}  // namespace esphome