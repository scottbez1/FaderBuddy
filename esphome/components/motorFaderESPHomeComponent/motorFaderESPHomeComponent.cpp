
#include "motorFaderESPHomeComponent.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "i2c_data.h"

namespace esphome {
namespace motorFaderESPHomeComponent {

static const char *const TAG = "motorFaderESPHomeComponent";

MotorFaderESPHomeComponent::MotorFaderESPHomeComponent() : PollingComponent(), i2c::I2CDevice() {
  // Initialize each layer with default haptic config (smooth)
  for (uint8_t i = 0; i < 8; i++) {
    layer_state_[i].expected_nonce = i;

    // Default haptic config: fully smooth (mode=0, detent_count=0)
    // Set nonce to match layer index
    layer_state_[i].haptic_config = make_haptic_config_internal_(
        i,    // nonce = layer index
        HAPTIC_NO_HAPTICS,  // mode = smooth
        0,    // detent_count
        0,    // detent_strength
        0     // target_position
    );
  }
}

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

    // Initialize firmware with layer 0 haptic config
    // Since we start on layer 0, send its config to ensure sync with firmware
    uint32_t cached_config = layer_state_[0].haptic_config;
    uint8_t nonce = (cached_config >> HAPTIC_NONCE_bp) & 0x07;
    uint8_t mode = (cached_config >> HAPTIC_MODE_bp) & 0x07;
    uint8_t detent_count = (cached_config >> HAPTIC_DETENT_COUNT_bp) & 0x0F;
    uint8_t detent_strength = (cached_config >> HAPTIC_DETENT_STRENGTH_bp) & 0x07;

    // Start at position 0 (no target position during init)
    uint32_t init_config = make_haptic_config_internal_(
        nonce, mode, detent_count, detent_strength, 0
    );

    ESP_LOGD(TAG, "Sending initial haptic config for layer 0 (mode=%d)", mode);
    write_haptic_config_(init_config);

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
            // Rate limit period has passed - trigger deferred USER-FACING value
            ESP_LOGI(TAG, "Deferred movement to %03d (user) on layer %d\n", deferred_value_, deferred_value_layer_);
            on_manual_move_->trigger(deferred_value_, deferred_value_layer_);
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

  // Convert raw bytes to state value
  uint32_t state = (
    (buffer[0] << 24)
    | (buffer[1] << 16)
    | (buffer[2] << 8)
    | (buffer[3])
    );

  Mode mode = static_cast<Mode>((state & STATE_MODE_bm) >> STATE_MODE_bp);
  uint16_t hw_position = (state & STATE_POSITION_bm) >> STATE_POSITION_bp;  // HARDWARE position from firmware
  uint8_t position_nonce = (state & STATE_POSITION_NONCE_bm) >> STATE_POSITION_NONCE_bp;
  bool touch = (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp;
  uint16_t raw_adc = (state & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp;
  uint8_t double_tap_nonce = (state & STATE_DOUBLE_TAP_NONCE_bm) >> STATE_DOUBLE_TAP_NONCE_bp;
  uint8_t reported_nonce = (state & STATE_HAPTIC_CONFIG_NONCE_bm) >> STATE_HAPTIC_CONFIG_NONCE_bp;

  // Check if state report is for the active layer
  if (pending_layer_switch_.has_value()) {
    // We're in the middle of a layer switch
    if (reported_nonce == layer_state_[active_layer_].expected_nonce) {
      // Firmware has confirmed the layer switch
      ESP_LOGD(TAG, "Layer switch to %d confirmed (nonce=%d)", active_layer_, reported_nonce);
      pending_layer_switch_.reset();
    } else if (millis() > layer_switch_timeout_) {
      // Timeout waiting for layer switch confirmation
      ESP_LOGW(TAG, "Layer switch timeout - expected nonce %d, got %d",
               layer_state_[active_layer_].expected_nonce, reported_nonce);
      pending_layer_switch_.reset();
      // Continue processing - maybe the nonce is correct now
    } else {
      // Still waiting for confirmation - ignore this state report
      ESP_LOGV(TAG, "Ignoring state report during layer switch (expected nonce %d, got %d)",
               layer_state_[active_layer_].expected_nonce, reported_nonce);
      return true;
    }
  }

  // Verify nonce matches active layer
  if (reported_nonce != layer_state_[active_layer_].expected_nonce) {
    ESP_LOGD(TAG, "Ignoring state report for wrong layer (expected nonce %d, got %d)",
             layer_state_[active_layer_].expected_nonce, reported_nonce);
    return true;
  }

  // Process state report for active layer
  layer_state_[active_layer_].has_been_initialized = true;

  if (hw_position != last_hw_position_ || position_nonce != last_position_nonce_) {
    if (mode == MODE_INPUT_ACTIVE || mode == MODE_INPUT_IDLE) {
      // Convert HARDWARE position to USER-FACING position for triggers
      uint8_t user_position = invert_ ? (255 - hw_position) : hw_position;

      // Handle rate limiting
      if (value_change_rate_limit_ == 0) {
        // No rate limiting - trigger immediately with USER-FACING position
        ESP_LOGI(TAG, "Movement to %03d (user) on layer %d\n", user_position, active_layer_);
        on_manual_move_->trigger(user_position, active_layer_);
      } else {
        uint32_t now = millis();
        uint32_t time_since_last_trigger = now - last_trigger_time_;

        if (time_since_last_trigger >= value_change_rate_limit_) {
          // Rate limit period has passed - trigger immediately with USER-FACING position
          ESP_LOGI(TAG, "Movement to %03d (user) on layer %d\n", user_position, active_layer_);
          on_manual_move_->trigger(user_position, active_layer_);
          last_trigger_time_ = now;
          has_deferred_value_ = false;
        } else {
          // Within rate limit window - defer the trigger with USER-FACING position
          deferred_value_ = user_position;
          deferred_value_layer_ = active_layer_;
          has_deferred_value_ = true;
        }
      }
    }
    last_hw_position_ = hw_position;
    last_position_nonce_ = position_nonce;

    // Update layer's last confirmed HARDWARE position
    layer_state_[active_layer_].last_confirmed_hw_position = hw_position;
    layer_state_[active_layer_].last_position_nonce = position_nonce;
  }

  // Check for touch state change
  if (touch != last_touch_) {
    ESP_LOGI(TAG, "Touch changed to %s on layer %d\n", touch ? "true" : "false", active_layer_);
    on_touch_change_->trigger(touch, active_layer_);
    last_touch_ = touch;
  }

  // Check for double tap
  if (double_tap_nonce != last_double_tap_nonce_) {
    ESP_LOGI(TAG, "Double tap detected on layer %d\n", active_layer_);
    on_double_tap_->trigger(active_layer_);
    last_double_tap_nonce_ = double_tap_nonce;
  }

  if (state != last_state_) {
    last_state_ = state;
    // ESP_LOGD(TAG, "State: %08x -- Current position: %03d, position_nonce: %d, touch: %01d, mode: %d, adc: %d\n", state, position, position_nonce, touch, mode, raw_adc);
    // ESP_LOGD(TAG, "BUFFER: %02x%02x%02x%02x", buffer[0], buffer[1], buffer[2], buffer[3]);
  }

  return true;
}

// Move fader to a specific position (USER-FACING position)
// position: 0-255 user-facing position (accounts for invert_ setting)
// layer: which layer to move (0-7, defaults to 0)
void MotorFaderESPHomeComponent::remote_move_to(uint8_t position, uint8_t layer) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  if (layer == active_layer_) {
    // Active layer - send USER-FACING position to hardware (will be converted to HW position)
    write_target_position_(position);
  } else {
    // Inactive layer - store USER-FACING position as restore position
    layer_state_[layer].restore_position = position;
    ESP_LOGD(TAG, "Stored user-facing position %d for inactive layer %d", position, layer);
  }
}

void MotorFaderESPHomeComponent::run_self_calibration() {
  uint8_t buffer = REG_SELF_CAL;
  auto write_result = this->write(&buffer, 1);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
      ESP_LOGE(TAG, "Failed to write self-calibration command: %d", write_result);
  }
}

// Helper function to construct haptic config value
uint32_t MotorFaderESPHomeComponent::make_haptic_config_internal_(
    uint8_t nonce,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength,
    uint8_t target_position) {

  uint32_t config = 0;
  config |= (nonce & 0x07) << HAPTIC_NONCE_bp;
  config |= (mode & 0x07) << HAPTIC_MODE_bp;
  config |= (detent_count & 0x0F) << HAPTIC_DETENT_COUNT_bp;
  config |= (detent_strength & 0x07) << HAPTIC_DETENT_STRENGTH_bp;
  config |= ((uint32_t)target_position & 0xFF) << HAPTIC_TARGET_POSITION_bp;
  return config;
}

// Helper function to write haptic config to hardware
// The target_position field in config should already be a HARDWARE position
void MotorFaderESPHomeComponent::write_haptic_config_(uint32_t config) {
  uint8_t buffer[5];
  buffer[0] = REG_HAPTIC_CONFIG;
  buffer[1] = (config >> 24) & 0xFF;
  buffer[2] = (config >> 16) & 0xFF;
  buffer[3] = (config >> 8) & 0xFF;
  buffer[4] = config & 0xFF;

  auto write_result = this->write(buffer, 5);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write haptic config: %d", write_result);
  }
}

// Helper function to write target position to hardware
// position: USER-FACING position (0-255), will be converted to HARDWARE position
void MotorFaderESPHomeComponent::write_target_position_(uint8_t position) {
  // Convert USER-FACING position to HARDWARE position
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

// Set the active layer
void MotorFaderESPHomeComponent::set_active_layer(uint8_t layer_index) {
  if (layer_index > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer_index);
    return;
  }

  if (layer_index == active_layer_) {
    return;  // Already on this layer
  }

  // Step 1: Capture current position from active layer (if it's been read)
  if (layer_state_[active_layer_].has_been_initialized) {
    // Convert HARDWARE position to USER-FACING position before storing
    uint8_t user_position = invert_ ? (255 - last_hw_position_) : last_hw_position_;
    layer_state_[active_layer_].restore_position = user_position;
    ESP_LOGD(TAG, "Stored user-facing position %d for layer %d", user_position, active_layer_);
  }

  // Step 2: Switch to new layer
  uint8_t old_layer = active_layer_;
  active_layer_ = layer_index;
  pending_layer_switch_ = layer_index;
  layer_switch_timeout_ = millis() + LAYER_SWITCH_TIMEOUT_MS;

  ESP_LOGI(TAG, "Switching from layer %d to layer %d", old_layer, layer_index);

  // Step 3: Send haptic config for new layer WITH target position included
  // Extract the haptic settings from the cached config (without target position)
  uint32_t cached_config = layer_state_[layer_index].haptic_config;
  uint8_t nonce = (cached_config >> HAPTIC_NONCE_bp) & 0x07;
  uint8_t mode = (cached_config >> HAPTIC_MODE_bp) & 0x07;
  uint8_t detent_count = (cached_config >> HAPTIC_DETENT_COUNT_bp) & 0x0F;
  uint8_t detent_strength = (cached_config >> HAPTIC_DETENT_STRENGTH_bp) & 0x07;

  // restore_position is USER-FACING, convert to HARDWARE position for firmware
  uint8_t user_target = layer_state_[layer_index].restore_position;
  uint8_t hw_target = invert_ ? (255 - user_target) : user_target;

  uint32_t config_with_target = make_haptic_config_internal_(
      nonce, mode, detent_count, detent_strength, hw_target
  );

  ESP_LOGD(TAG, "Moving to restore position %d (hw: %d) for layer %d via haptic config",
           user_target, hw_target, layer_index);
  write_haptic_config_(config_with_target);
}

// Get the current target/restore position for a layer
// Returns USER-FACING position (0-255)
uint8_t MotorFaderESPHomeComponent::get_position(uint8_t layer) const {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return 0;
  }
  return layer_state_[layer].restore_position;  // Already USER-FACING
}

// Set haptic configuration for a specific layer
// target_position: USER-FACING position (0-255), only used if layer is currently active
void MotorFaderESPHomeComponent::set_layer_haptic_config(
    uint8_t layer,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength,
    uint8_t target_position) {

  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  // Build haptic config with nonce matching layer index
  // Note: we store the config WITHOUT target position (keep it 0 in cache)
  // Target position will be added during layer switches or when applying to active layer
  uint32_t config = make_haptic_config_internal_(
      layer,            // nonce = layer index
      mode,
      detent_count,
      detent_strength,
      0                 // Don't include target in cached config
  );

  // Cache the config for this layer
  layer_state_[layer].haptic_config = config;

  ESP_LOGD(TAG, "Set haptic config for layer %d: mode=%d, detents=%d, strength=%d",
           layer, mode, detent_count, detent_strength);

  // If this is the active layer, send config immediately
  // Convert USER-FACING target_position to HARDWARE position if provided
  if (layer == active_layer_) {
    uint8_t hw_target = invert_ ? (255 - target_position) : target_position;
    uint32_t config_with_target = make_haptic_config_internal_(
        layer, mode, detent_count, detent_strength, hw_target
    );
    write_haptic_config_(config_with_target);
  }
}

}  // namespace motorFaderESPHomeComponent
}  // namespace esphome