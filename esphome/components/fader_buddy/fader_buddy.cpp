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
  uint8_t buffer = 0;
  auto read_result = this->write_read(&reg, 1, &buffer, 1);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Init: failed to read VERSION register: %d", read_result);
    this->mark_failed();
    return;
  }

  // Older protocols really are incompatible - the register layout differs - but
  // a NEWER one is not, because bumps are only made for changes a host can
  // ignore. Failing on those would brick this host on a fader that merely got
  // updated, so warn and carry on instead.
  if (buffer < I2C_PROTOCOL_VERSION) {
    ESP_LOGE(TAG, "Init: Incompatible I2C protocol version. Expected at least %d but got %d",
             I2C_PROTOCOL_VERSION, buffer);
    this->mark_failed();
    return;
  }
  if (buffer > I2C_PROTOCOL_VERSION) {
    ESP_LOGW(TAG, "Fader reports protocol v%d, newer than the v%d this component was built against. "
                  "Continuing, but consider updating the component.", buffer, I2C_PROTOCOL_VERSION);
  }

  ESP_LOGCONFIG(TAG, "FaderBuddy initialized (component %s, protocol v%d)",
                FADER_BUDDY_COMPONENT_VERSION, buffer);

  // Read the chip serial number once (static factory ID) and publish it.
  read_serial_number_();
  read_firmware_version_();

  // Flag a config that asks for something this fader's firmware cannot do, at
  // startup rather than waiting for the first move to warn.
  if (!speed_supported_) {
    for (uint8_t i = 0; i < 8; i++) {
      if (layer_states_[i].default_speed != LAYER_SPEED_FULL) {
        ESP_LOGW(TAG, "Layer %d sets default_speed, but this fader's firmware does not support move "
                      "speed. Moves will run at full speed. Update the fader firmware to %d.%d or newer.",
                 i, FW_VERSION_MOVE_SPEED >> 8, FW_VERSION_MOVE_SPEED & 0xFF);
        break;
      }
    }
  }

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

  ESP_LOGCONFIG(TAG, "  Component Version: %s", FADER_BUDDY_COMPONENT_VERSION);
  if (this->firmware_version_ == FW_VERSION_NONE) {
    ESP_LOGCONFIG(TAG, "  Firmware Version: 1.0 or older (does not report a version)");
  } else {
    ESP_LOGCONFIG(TAG, "  Firmware Version: %d.%d", this->firmware_version_ >> 8,
                  this->firmware_version_ & 0xFF);
  }

  if (!this->serial_number_.empty()) {
    ESP_LOGCONFIG(TAG, "  Serial Number: %s", this->serial_number_.c_str());
  }
  LOG_TEXT_SENSOR("  ", "Serial Number", this->serial_text_sensor_);

  LOG_UPDATE_INTERVAL(this);
}

// Read the 10-byte chip serial number and cache it as an uppercase hex string.
// The serial is a static factory ID, so this only needs to run once at setup.
void FaderBuddy::read_serial_number_() {
  uint8_t reg = REG_SERIAL;
  uint8_t serial[10];
  auto read_result = this->write_read(&reg, 1, serial, sizeof(serial));
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGW(TAG, "Failed to read serial number: %d", read_result);
    return;
  }

  char buf[sizeof(serial) * 2 + 1];
  for (size_t i = 0; i < sizeof(serial); i++) {
    sprintf(buf + i * 2, "%02X", serial[i]);
  }
  this->serial_number_ = buf;
  ESP_LOGCONFIG(TAG, "Serial number: %s", this->serial_number_.c_str());

  if (this->serial_text_sensor_ != nullptr) {
    this->serial_text_sensor_->publish_state(this->serial_number_);
  }
}

// Read the reported firmware version and derive what this fader can do.
// Firmware predating the register leaves the bus undriven, so the read
// succeeds and returns 0xFFFF rather than failing - hence checking the value,
// not just the error code.
void FaderBuddy::read_firmware_version_() {
  uint8_t reg = REG_FW_VERSION;
  uint8_t buffer[2] = {0xFF, 0xFF};
  auto read_result = this->write_read(&reg, 1, buffer, sizeof(buffer));
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGW(TAG, "Failed to read firmware version: %d; assuming pre-1.1 firmware", read_result);
    this->firmware_version_ = FW_VERSION_NONE;
  } else {
    this->firmware_version_ = ((uint16_t) buffer[0] << 8) | buffer[1];
  }

  if (this->firmware_version_ == FW_VERSION_NONE || this->firmware_version_ == 0) {
    this->firmware_version_ = FW_VERSION_NONE;
    ESP_LOGCONFIG(TAG, "Fader firmware: 1.0 or older (no version register)");
  } else {
    ESP_LOGCONFIG(TAG, "Fader firmware: %d.%d", this->firmware_version_ >> 8,
                  this->firmware_version_ & 0xFF);
  }

  this->speed_supported_ = this->firmware_version_ != FW_VERSION_NONE &&
                           this->firmware_version_ >= FW_VERSION_MOVE_SPEED;
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

  auto read_result = this->write_read(&reg, 1, buffer, 4);
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
    // ESP_LOGD(TAG, "State: %08x -- Current position: %03d, position_nonce: %d, touch: %01d, mode: %d, adc: %d, double_tap_nonce: %d\n", (unsigned int) state, hw_position, position_nonce, touch, mode, raw_adc, double_tap_nonce);
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
      ESP_LOGD(TAG, "State: %08x -- Current position: %03d, position_nonce: %d, active_layer: %d, touch: %01d, mode: %d, adc: %d, double_tap_nonce: %d\n", (unsigned int) state, hw_position, position_nonce, active_layer, touch, mode, raw_adc, double_tap_nonce);
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

  auto read_result = this->write_read(&reg, 1, &buffer, 1);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to read active layer: %d", read_result);
    return 0;
  }

  return buffer & 0x07;
}

// Move fader to a specific position (Protocol v5: use REG_LAYER_TARGET)
// position: USER-FACING position (0-255)
// layer: which layer to move (0-7)
// Uses the layer's configured default speed.
void FaderBuddy::remote_move_to(uint8_t position, uint8_t layer) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }
  this->remote_move_to(position, layer, layer_states_[layer].default_speed);
}

// speed: unitless 0-255 (LAYER_SPEED_FULL = full speed, 0 = slowest smooth motion)
void FaderBuddy::remote_move_to(uint8_t position, uint8_t layer, uint8_t speed) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  // Convert USER-FACING position to HARDWARE position
  uint8_t hw_position = invert_ ? (255 - position) : position;

  // Firmware without the speed byte drops a 4-byte write entirely, so fall
  // back to a full-speed move rather than letting the fader not move at all.
  if (speed != LAYER_SPEED_FULL && !speed_supported_) {
    if (!warned_speed_unsupported_) {
      warned_speed_unsupported_ = true;
      ESP_LOGW(TAG, "Requested move speed %d, but this fader's firmware does not support it "
                    "(needs %d.%d or newer). Moving at full speed instead. "
                    "This warning is logged once.",
               speed, FW_VERSION_MOVE_SPEED >> 8, FW_VERSION_MOVE_SPEED & 0xFF);
    }
    speed = LAYER_SPEED_FULL;
  }

  // Write to firmware using layer-addressed protocol. At full speed send the
  // original 3-byte write: it is equivalent, and firmware predating the speed
  // byte ignores a 4-byte write entirely rather than moving.
  uint8_t buffer[] = {REG_LAYER_TARGET, layer, hw_position, speed};
  size_t len = (speed == LAYER_SPEED_FULL) ? 3 : 4;
  if (write_with_retry_(buffer, len)) {
    ESP_LOGD(TAG, "Set layer %d target to %d (user position) at speed %d", layer, position, speed);
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

  // Read restore position for that layer
  uint8_t read_buffer = 0;
  auto read_result = this->write_read(write_buffer, 2, &read_buffer, 1);
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

void FaderBuddy::set_layer_default_speed(uint8_t layer, uint8_t speed) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }
  layer_states_[layer].default_speed = speed;
}

uint8_t FaderBuddy::get_layer_default_speed(uint8_t layer) const {
  if (layer > 7) {
    return LAYER_SPEED_FULL;
  }
  return layer_states_[layer].default_speed;
}

void FaderBuddy::run_self_calibration() {
  uint8_t buffer = REG_SELF_CAL;
  if (!write_with_retry_(&buffer, 1)) {
    ESP_LOGE(TAG, "Failed to write self-calibration command");
  }
}

}  // namespace fader_buddy
}  // namespace esphome
