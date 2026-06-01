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

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "i2c_data.h"

namespace esphome {
namespace fader_buddy {

static const char *const TAG = "fader_buddy";

FaderBuddy::FaderBuddy() : PollingComponent(), i2c::I2CDevice() {
  // Protocol v5: No layer state initialization needed - firmware manages layers
}

void FaderBuddy::setup() {
  ESP_LOGCONFIG(TAG, "Setting up FaderBuddy...");

  // Check protocol version
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
    ESP_LOGE(TAG, "Init: Incompatible I2C protocol version. Expected %d but got %d", I2C_PROTOCOL_VERSION, buffer);
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "FaderBuddy initialized (protocol v%d)", buffer);

  // Send initial haptic configurations to firmware
  for (uint8_t i = 0; i < 8; i++) {
    if (initial_haptic_configs_[i].valid) {
      ESP_LOGCONFIG(TAG, "Sending initial haptic config for layer %d: mode=%d, detents=%d, strength=%d",
                    i, initial_haptic_configs_[i].mode, initial_haptic_configs_[i].detent_count,
                    initial_haptic_configs_[i].detent_strength);
      send_layer_haptic_config_(i, initial_haptic_configs_[i].mode,
                                initial_haptic_configs_[i].detent_count,
                                initial_haptic_configs_[i].detent_strength);
    }
  }
  set_active_layer(0);

  if (this->get_update_interval() < App.get_loop_interval()) {
    high_freq_.start();
  }
}

void FaderBuddy::dump_config() {
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication failed");
  }

  LOG_UPDATE_INTERVAL(this);
}

float FaderBuddy::get_setup_priority() const { return setup_priority::DATA; }

void FaderBuddy::update() {
  // Check all layers for deferred triggers to fire
  for (uint8_t layer = 0; layer < 8; layer++) {
    if (layer_states_[layer].has_deferred_value && layer_states_[layer].value_change_min_interval > 0) {
      uint32_t now = millis();
      uint32_t time_since_last_trigger = now - layer_states_[layer].last_trigger_time;

      if (time_since_last_trigger >= layer_states_[layer].value_change_min_interval) {
        // Min interval period has passed - trigger deferred USER-FACING value
        ESP_LOGI(TAG, "Deferred movement to %03d (user) on layer %d\n", layer_states_[layer].deferred_value, layer);
        layer_states_[layer].last_trigger_time = now;
        layer_states_[layer].has_deferred_value = false;
        on_manual_move_->trigger(layer_states_[layer].deferred_value, layer);
      }
    }
  }

  if (!read_sensor_data_()) {
    ESP_LOGW(TAG, "Failed to read from sensor.");
  }
}

bool FaderBuddy::read_sensor_data_() {
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

  // Parse STATE register
  uint32_t state = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
                   ((uint32_t)buffer[2] << 8) | buffer[3];

  Mode mode = static_cast<Mode>((state & STATE_MODE_bm) >> STATE_MODE_bp);
  uint16_t hw_position = (state & STATE_POSITION_bm) >> STATE_POSITION_bp;
  uint8_t position_nonce = (state & STATE_POSITION_NONCE_bm) >> STATE_POSITION_NONCE_bp;
  bool touch = (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp;
  uint16_t raw_adc = (state & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp;
  uint8_t double_tap_nonce = (state & STATE_DOUBLE_TAP_NONCE_bm) >> STATE_DOUBLE_TAP_NONCE_bp;
  uint8_t active_layer = (state & STATE_ACTIVE_LAYER_bm) >> STATE_ACTIVE_LAYER_bp;

  if (state != last_state_) {
    last_state_ = state;
    // ESP_LOGD(TAG, "State: %08x -- Current position: %03d, position_nonce: %d, touch: %01d, mode: %d, adc: %d, double_tap_nonce: %d\n", state, hw_position, position_nonce, touch, mode, raw_adc, double_tap_nonce);
  }

  // Check for position changes (per-layer tracking)
  if (hw_position != layer_states_[active_layer].last_hw_position ||
      position_nonce != layer_states_[active_layer].last_position_nonce) {
    layer_states_[active_layer].last_hw_position = hw_position;
    layer_states_[active_layer].last_position_nonce = position_nonce;

    // Fire raw position update immediately, regardless of mode or rate limiting
    uint8_t user_position = invert_ ? (255 - hw_position) : hw_position;
    on_raw_position_update_->trigger(user_position, active_layer);

    if (mode == MODE_INPUT_ACTIVE || mode == MODE_INPUT_IDLE) {
      // Convert HARDWARE position to USER-FACING position
      layer_states_[active_layer].deferred_value = user_position;
      layer_states_[active_layer].has_deferred_value = true;
    }
  }

  if (layer_states_[active_layer].has_deferred_value) {
    uint32_t now = millis();
    uint32_t time_since_last_trigger = now - layer_states_[active_layer].last_trigger_time;
    if (time_since_last_trigger >= layer_states_[active_layer].value_change_min_interval) {
      // Min interval period has passed
      ESP_LOGD(TAG, "State: %08x -- Current position: %03d, position_nonce: %d, active_layer: %d, touch: %01d, mode: %d, adc: %d, double_tap_nonce: %d\n", state, hw_position, position_nonce, active_layer, touch, mode, raw_adc, double_tap_nonce);
      ESP_LOGI(TAG, "Movement to %03d (user) on layer %d\n", layer_states_[active_layer].deferred_value, active_layer);
      layer_states_[active_layer].last_trigger_time = now;
      layer_states_[active_layer].has_deferred_value = false;
      on_manual_move_->trigger(layer_states_[active_layer].deferred_value, active_layer);
    }
  }

  // Check for touch state change
  if (touch != last_touch_) {
    ESP_LOGI(TAG, "Touch changed to %s on layer %d\n", touch ? "true" : "false", active_layer);
    last_touch_ = touch;
    on_touch_change_->trigger(touch, active_layer);
  }

  // Check for double tap
  if (double_tap_nonce != last_double_tap_nonce_) {
    ESP_LOGI(TAG, "Double tap detected on layer %d -- %d\n", active_layer, double_tap_nonce);
    last_double_tap_nonce_ = double_tap_nonce;
    on_double_tap_->trigger(active_layer);
  }

  return true;
}

// Set the active layer (Protocol v5: simple write to firmware)
void FaderBuddy::set_active_layer(uint8_t layer_index) {
  if (layer_index > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer_index);
    return;
  }

  // Simple write to firmware
  uint8_t buffer[] = {REG_ACTIVE_LAYER, layer_index};
  if (write_with_retry_(buffer, 2)) {
    ESP_LOGI(TAG, "Set active layer to %d", layer_index);
  } else {
    ESP_LOGE(TAG, "Failed to set active layer %d", layer_index);
  }
}

// Get the active layer (Protocol v5: read from firmware)
uint8_t FaderBuddy::get_active_layer() const {
  uint8_t reg = REG_ACTIVE_LAYER;
  uint8_t buffer = 0;

  auto write_result = this->write(&reg, 1, false);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write register address: %d", write_result);
    return 0;  // Default to layer 0 on error
  }

  auto read_result = this->read(&buffer, 1);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to read active layer: %d", read_result);
    return 0;
  }

  return buffer & 0x07;
}

// Move fader to a specific position (Protocol v5: use REG_LAYER_TARGET)
// position: USER-FACING position (0-255)
// layer: which layer to move (0-7)
void FaderBuddy::remote_move_to(uint8_t position, uint8_t layer) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  // Convert USER-FACING position to HARDWARE position
  uint8_t hw_position = invert_ ? (255 - position) : position;

  // Write to firmware using layer-addressed protocol
  uint8_t buffer[] = {REG_LAYER_TARGET, layer, hw_position};
  if (write_with_retry_(buffer, 3)) {
    ESP_LOGD(TAG, "Set layer %d target to %d (user position)", layer, position);
  } else {
    ESP_LOGE(TAG, "Failed to write layer %d target", layer);
  }
}

// Get the restore position for a layer (Protocol v5: layer-addressed read)
// Returns USER-FACING position (0-255)
uint8_t FaderBuddy::get_position(uint8_t layer) const {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return 0;
  }

  // Write layer index to query
  uint8_t write_buffer[] = {REG_LAYER_TARGET, layer};
  auto write_result = this->write(write_buffer, 2, false);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write layer index: %d", write_result);
    return 0;
  }

  // Read restore position for that layer
  uint8_t read_buffer = 0;
  auto read_result = this->read(&read_buffer, 1);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to read layer position: %d", read_result);
    return 0;
  }

  // Convert HARDWARE position to USER-FACING position
  return invert_ ? (255 - read_buffer) : read_buffer;
}

// Helper: Write to I2C with retries for transient failures
bool FaderBuddy::write_with_retry_(const uint8_t *data, size_t len, uint8_t retries) {
  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    auto result = this->write(data, len);
    if (result == esphome::i2c::ErrorCode::NO_ERROR) {
      return true;
    }
    if (attempt < retries - 1) {
      ESP_LOGD(TAG, "I2C write failed (attempt %d/%d), retrying...", attempt + 1, retries);
      delay(1);  // Small delay before retry
    }
  }
  ESP_LOGE(TAG, "I2C write failed after %d attempts", retries);
  return false;
}

// Helper: Send haptic config to firmware over I2C (Protocol v5: 16-bit format)
void FaderBuddy::send_layer_haptic_config_(
    uint8_t layer,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength) {

  // Build haptic config (16 bits) - Protocol v5 format using constants from i2c_data.h
  uint16_t config = 0;
  config |= (mode << HAPTIC_MODE_bp) & HAPTIC_MODE_bm;
  config |= (detent_count << HAPTIC_DETENT_COUNT_bp) & HAPTIC_DETENT_COUNT_bm;
  config |= (detent_strength << HAPTIC_DETENT_STRENGTH_bp) & HAPTIC_DETENT_STRENGTH_bm;

  // Write to firmware (4 bytes: register + layer + config big-endian)
  uint8_t buffer[4];
  buffer[0] = REG_LAYER_HAPTIC_CONFIG;
  buffer[1] = layer;
  buffer[2] = (config >> 8) & 0xFF;  // High byte
  buffer[3] = config & 0xFF;         // Low byte

  if (!write_with_retry_(buffer, 4)) {
    ESP_LOGE(TAG, "Failed to write layer haptic config for layer %d", layer);
  }
}

// Store initial haptic config (called only from codegen, before setup)
void FaderBuddy::store_initial_layer_haptic_config(
    uint8_t layer,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength) {

  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  initial_haptic_configs_[layer].layer = layer;
  initial_haptic_configs_[layer].mode = mode;
  initial_haptic_configs_[layer].detent_count = detent_count;
  initial_haptic_configs_[layer].detent_strength = detent_strength;
  initial_haptic_configs_[layer].valid = true;
}

// Set haptic configuration at runtime (sends immediately to firmware)
void FaderBuddy::set_layer_haptic_config(
    uint8_t layer,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength) {

  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  ESP_LOGD(TAG, "Set haptic config for layer %d: mode=%d, detents=%d, strength=%d",
           layer, mode, detent_count, detent_strength);
  send_layer_haptic_config_(layer, mode, detent_count, detent_strength);
}

void FaderBuddy::set_layer_value_change_min_interval(uint8_t layer, uint32_t min_interval_ms) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }
  layer_states_[layer].value_change_min_interval = min_interval_ms;
}

void FaderBuddy::run_self_calibration() {
  uint8_t buffer = REG_SELF_CAL;
  if (!write_with_retry_(&buffer, 1)) {
    ESP_LOGE(TAG, "Failed to write self-calibration command");
  }
}

}  // namespace fader_buddy
}  // namespace esphome
