
#include "motorFaderESPHomeComponent.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "../../protocol/i2c_data.h"

namespace esphome {
namespace motorFaderESPHomeComponent {

static const char *const TAG = "motorFaderESPHomeComponent";

void MotorFaderESPHomeComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MotorFaderESPHomeComponent...");
//   // Optional: test communication
//   if (!this->is_device_ready()) {
//     ESP_LOGE(TAG, "Sensor not responding at 0x%02X", this->address_);
//     this->mark_failed();
//     return;
//   }

    uint8_t reg = REG_VERSION;
    auto write_result = this->write(&reg, 1, false);
    if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
        ESP_LOGE(TAG, "Init: failed to write register address for VERSION: %d", write_result);
        this->mark_failed();
        return;
    }

    uint8_t buffer = 0;
    auto read_result = this->read(&buffer, 1);
    if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
        ESP_LOGE(TAG, "Init: failed to read VERSION register: %d", read_result);
        this->mark_failed();
        return;
    }

    if (buffer != I2C_PROTOCOL_VERSION) {
        ESP_LOGE(TAG, "Init: Incompatible I2C protocol version. Expected %d but got %d", I2C_PROTOCOL_VERSION, read_result);
        this->mark_failed();
        return;
    }

    if (this->get_update_interval() < App.get_loop_interval()) {
        high_freq_.start();
    }
}

void MotorFaderESPHomeComponent::dump_config() {
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication failed");
  }

  LOG_UPDATE_INTERVAL(this);
}

float MotorFaderESPHomeComponent::get_setup_priority() const { return setup_priority::DATA; }

void MotorFaderESPHomeComponent::update() {
    // Check if we have a deferred trigger to fire
    if (has_deferred_value_ && value_change_rate_limit_ > 0) {
        uint32_t now = millis();
        uint32_t time_since_last_trigger = now - last_trigger_time_;

        if (time_since_last_trigger >= value_change_rate_limit_) {
            // Rate limit period has passed - trigger deferred value
            ESP_LOGI(TAG, "Deferred movement to %03d\n", deferred_value_);
            on_manual_move_->trigger(deferred_value_);
            last_trigger_time_ = now;
            has_deferred_value_ = false;
        }
    }

    if (!read_sensor_data_()) {
        ESP_LOGW(TAG, "Failed to read from sensor.");
    }
}

bool MotorFaderESPHomeComponent::read_sensor_data_() {
  uint8_t reg = REG_STATE;
  uint8_t buffer[4];

  auto write_result = this->write(&reg, 1, false);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write register address: %d", write_result);
    return false;
  }

  auto read_result = this->read(buffer, 4);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to read data: %d", read_result);
    return false;
  }

  // Convert raw bytes to a float value (example logic)
  uint32_t state = (
    (buffer[0] << 24)
    | (buffer[1] << 16)
    | (buffer[2] << 8)
    | (buffer[3])
    );

  Mode mode = static_cast<Mode>((state & STATE_MODE_bm) >> STATE_MODE_bp);

  uint16_t position = (state & STATE_POSITION_bm) >> STATE_POSITION_bp;
  uint8_t position_nonce = (state & STATE_POSITION_NONCE_bm) >> STATE_POSITION_NONCE_bp;
  bool touch = (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp;
  uint16_t raw_adc = (state & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp;
  uint8_t double_tap_nonce = (state & STATE_DOUBLE_TAP_NONCE_bm) >> STATE_DOUBLE_TAP_NONCE_bp;

  if (position != last_position_ || position_nonce != last_position_nonce_) {
    if (mode == MODE_INPUT_ACTIVE || mode == MODE_INPUT_IDLE) {
      uint8_t reported_position = invert_ ? (255 - position) : position;

      // Handle rate limiting
      if (value_change_rate_limit_ == 0) {
        // No rate limiting - trigger immediately
        ESP_LOGI(TAG, "Movement to %03d\n", reported_position);
        on_manual_move_->trigger(reported_position);
      } else {
        uint32_t now = millis();
        uint32_t time_since_last_trigger = now - last_trigger_time_;

        if (time_since_last_trigger >= value_change_rate_limit_) {
          // Rate limit period has passed - trigger immediately
          ESP_LOGI(TAG, "Movement to %03d\n", reported_position);
          on_manual_move_->trigger(reported_position);
          last_trigger_time_ = now;
          has_deferred_value_ = false;
        } else {
          // Within rate limit window - defer the trigger
          // The deferred value will be triggered in the next update() cycle
          deferred_value_ = reported_position;
          has_deferred_value_ = true;
        }
      }
    }
    last_position_ = position;
    last_position_nonce_ = position_nonce;
  }

  // Check for touch state change
  if (touch != last_touch_) {
    ESP_LOGI(TAG, "Touch changed to %s\n", touch ? "true" : "false");
    on_touch_change_->trigger(touch);
    last_touch_ = touch;
  }

  // Check for double tap
  if (double_tap_nonce != last_double_tap_nonce_) {
    ESP_LOGI(TAG, "Double tap detected\n");
    on_double_tap_->trigger();
    last_double_tap_nonce_ = double_tap_nonce;
  }

  if (state != last_state_) {
    last_state_ = state;
    // ESP_LOGD(TAG, "State: %08x -- Current position: %03d, position_nonce: %d, touch: %01d, mode: %d, adc: %d\n", state, position, position_nonce, touch, mode, raw_adc);
    // ESP_LOGD(TAG, "BUFFER: %02x%02x%02x%02x", buffer[0], buffer[1], buffer[2], buffer[3]);
  }

  return true;
}

void MotorFaderESPHomeComponent::remote_move_to(uint8_t position) {
  uint8_t hw_position = invert_ ? (255 - position) : position;
  uint8_t buffer[] = {REG_TARGET, hw_position};

    for (uint8_t i = 0; i < 2; i++) {
        auto write_result = this->write(buffer, 2);
        if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
            ESP_LOGE(TAG, "Failed to write target: %d", write_result);
            continue;
        }

        // Read back to validate target was set
        uint8_t reg = REG_TARGET;
        write_result = this->write(&reg, 1, false);
        if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
            ESP_LOGE(TAG, "Failed to write register address: %d", write_result);
            continue;
        }

        auto read_result = this->read(buffer, 2);
        if (read_result == esphome::i2c::ErrorCode::NO_ERROR) {
            return;
        }
        ESP_LOGE(TAG, "Failed to read data: %d", read_result);
    }
    ESP_LOGE(TAG, "Gave up trying to write/validate target register");
}

void MotorFaderESPHomeComponent::run_self_calibration() {
  uint8_t buffer = REG_SELF_CAL;
  auto write_result = this->write(&buffer, 1);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
      ESP_LOGE(TAG, "Failed to write self-calibration command: %d", write_result);
  }
}

}  // namespace motorFaderESPHomeComponent
}  // namespace esphome