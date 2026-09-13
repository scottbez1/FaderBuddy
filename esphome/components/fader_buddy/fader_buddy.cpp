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

#include <cstdio>

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "i2c_data.h"
#include "bootloader_protocol.h"

namespace esphome {
namespace fader_buddy {

static const char *const TAG = "fader_buddy";

bool FaderBuddy::s_update_in_progress = false;

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
  read_motor_calibration_();

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

  if (firmware_image_ != nullptr) {
    // Keyed by address + target version: switching to a differently-versioned
    // packaged image naturally starts a fresh (zero) attempt count for the new
    // key, with no explicit reset needed.
    uint32_t hash = fnv1_hash("fader_buddy_update_attempts_" + std::to_string(this->get_i2c_address()) +
                              "_v" + std::to_string(firmware_fw_version_));
    update_attempts_pref_ = global_preferences->make_preference<uint8_t>(hash, true);
  }

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
  LOG_TEXT_SENSOR("  ", "Firmware Version", this->firmware_text_sensor_);

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

  char version[16];
  if (this->firmware_version_ == FW_VERSION_NONE || this->firmware_version_ == 0) {
    this->firmware_version_ = FW_VERSION_NONE;
    // Pre-1.1 firmware has no version register, so this is the most specific
    // thing that can be said about it.
    snprintf(version, sizeof(version), "1.0 or older");
    ESP_LOGCONFIG(TAG, "Fader firmware: %s (no version register)", version);
  } else {
    snprintf(version, sizeof(version), "%d.%d", this->firmware_version_ >> 8,
             this->firmware_version_ & 0xFF);
    ESP_LOGCONFIG(TAG, "Fader firmware: %s", version);
  }

  if (this->firmware_text_sensor_ != nullptr) {
    this->firmware_text_sensor_->publish_state(version);
  }

  this->speed_supported_ = this->firmware_version_ != FW_VERSION_NONE &&
                           this->firmware_version_ >= FW_VERSION_MOVE_SPEED;
}

// Log what self-calibration measured about this fader's motor, and the
// feedforward it derived. Diagnostic only - the fader needs nothing from the
// host here - but these are the numbers to look at when a fader hunts or
// settles slowly, and on a bench with no test jig attached this log is the
// only way to see them.
void FaderBuddy::read_motor_calibration_() {
  if (this->firmware_version_ == FW_VERSION_NONE ||
      this->firmware_version_ < FW_VERSION_MOTOR_CAL) {
    return;  // Older firmware has no such register; nothing to report
  }

  uint8_t reg = REG_MOTOR_CAL;
  uint8_t b[12] = {0};
  if (this->write_read(&reg, 1, b, sizeof(b)) != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGW(TAG, "Failed to read motor calibration");
    return;
  }

  uint16_t vel_min = ((uint16_t) b[9] << 8) | b[10];

  if (b[0] == 0) {
    ESP_LOGCONFIG(TAG, "Motor: not characterised, using the default plant model "
                       "(vel_min %u ADC/s, deadband %d). Run self-calibration to measure this unit.",
                  vel_min, b[11]);
    return;
  }

  ESP_LOGCONFIG(TAG, "Motor: breakaway %d/%d duty, k %d/%d ADC/s per duty, "
                     "jump %u/%u ADC/s (rising/falling)",
                b[1], b[2], b[3], b[4],
                ((uint16_t) b[5] << 8) | b[6], ((uint16_t) b[7] << 8) | b[8]);
  ESP_LOGCONFIG(TAG, "Motor: vel_min %u ADC/s, deadband %d ADC counts",
                vel_min, b[11]);
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

  // The fader's mode changes on its own - self-calibration finishing, a move
  // timing out into MODE_ERROR - and nothing else here reports that.
  if (mode != this->last_mode_) {
    Mode previous = this->last_mode_;
    this->last_mode_ = mode;

    if (mode == MODE_SELF_CALIBRATION) {
      ESP_LOGI(TAG, "Self-calibration started");
    } else if (previous == MODE_SELF_CALIBRATION) {
      if (mode == MODE_ERROR) {
        ESP_LOGE(TAG, "Self-calibration failed: the endpoint sweep found no usable travel. "
                      "Check the motor and potentiometer wiring. The fader ignores position "
                      "commands until the error is cleared.");
      } else {
        // Re-read what it measured. The startup log ran before this, so these
        // are the only numbers that reflect the run that just finished.
        ESP_LOGI(TAG, "Self-calibration complete");
        this->read_motor_calibration_();
      }
    } else if (mode == MODE_ERROR) {
      ESP_LOGW(TAG, "Fader latched MODE_ERROR (a move did not reach its target). "
                    "Position commands are ignored until the error is cleared.");
    } else {
      ESP_LOGD(TAG, "Mode %d -> %d", previous, mode);
    }
  }

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

  // Always send the speed when the firmware understands it, full speed
  // included. A 3-byte write leaves the layer's STORED speed alone, so after
  // any slower move it would silently re-apply that old limit - it is only
  // equivalent to a full-speed move if the layer was already at full speed.
  // Firmware predating the byte ignores a 4-byte write entirely, so that case
  // (where speed has been forced to LAYER_SPEED_FULL above) still sends 3.
  uint8_t buffer[] = {REG_LAYER_TARGET, layer, hw_position, speed};
  size_t len = speed_supported_ ? 4 : 3;
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

// ---------------------------------------------------------------------------
// Firmware update over the I2C bootloader (ABOUT_I2C_BOOTLOADER.md section 11)
// ---------------------------------------------------------------------------

void FaderBuddy::set_firmware_image(const uint8_t *image, uint32_t length, uint16_t image_crc16,
                                    uint16_t fw_version) {
  firmware_image_ = image;
  firmware_image_length_ = length;
  firmware_image_crc16_ = image_crc16;
  firmware_fw_version_ = fw_version;
}

uint8_t FaderBuddy::get_last_mode_() const { return (last_state_ & STATE_MODE_bm) >> STATE_MODE_bp; }

void FaderBuddy::update_firmware() {
  if (firmware_image_ == nullptr) {
    ESP_LOGE(TAG, "update_firmware: no firmware_image configured");
    on_firmware_update_result_->trigger(false, "no firmware_image configured");
    return;
  }
  if (s_update_in_progress) {
    ESP_LOGW(TAG, "update_firmware: another fader's update is already in progress, skipping");
    on_firmware_update_result_->trigger(false, "another update already in progress");
    return;
  }

  // Don't interrupt the user: wait (bounded) for the fader to go idle before taking the bus.
  uint32_t wait_start = millis();
  while (get_last_mode_() == MODE_INPUT_ACTIVE && millis() - wait_start < 5000) {
    delay(50);
    read_sensor_data_();
  }
  if (get_last_mode_() == MODE_INPUT_ACTIVE) {
    ESP_LOGW(TAG, "update_firmware: fader still in use, deferring");
    on_firmware_update_result_->trigger(false, "fader in use");
    return;
  }

  // Register 0x00 is the universal "who are you" probe: a valid protocol version
  // means the app is running (and REG_FW_VERSION is meaningful); BL_VERSION_MARKER
  // means the bootloader is already resident. Probe that first -- REG_FW_VERSION
  // (0x11) is app-only and undefined in the bootloader, so trying it blind first
  // risks misreading garbage as a version.
  uint8_t probe;
  bool responded = bl_read_version_byte_(probe);
  if (!responded) {
    ESP_LOGE(TAG, "update_firmware: device not responding");
    on_firmware_update_result_->trigger(false, "device not responding");
    return;
  }
  bool bootloader_resident = probe == BL_VERSION_MARKER;
  uint16_t current_version = 0;
  bool have_version = !bootloader_resident && read_fw_version_(current_version);

  if (have_version && current_version == firmware_fw_version_) {
    ESP_LOGI(TAG, "update_firmware: already at v%u, nothing to do", current_version);
    uint8_t zero = 0;
    update_attempts_pref_.save(&zero);
    on_firmware_update_result_->trigger(true, "");
    return;
  }

  // Firmware predating FW_VERSION_BOOTLOADER_ENTRY ignores REG_ENTER_BOOTLOADER,
  // so there is no way to reach the bootloader over I2C -- and no bootloader
  // behind it to reach. Installing one is a fuse write, hence UPDI-only. Refuse
  // here rather than writing the magic and timing out waiting for a marker that
  // will never appear.
  // FW_VERSION_NONE (0xFFFF) is what unversioned firmware reads back as, and
  // numerically exceeds the threshold -- it has to be excluded explicitly.
  if (have_version && (current_version == FW_VERSION_NONE ||
                       current_version < FW_VERSION_BOOTLOADER_ENTRY)) {
    ESP_LOGE(TAG, "update_firmware: firmware v%u.%u predates I2C bootloader entry "
                  "(needs >= v%u.%u); one-time UPDI migration required",
             current_version >> 8, current_version & 0xFF,
             FW_VERSION_BOOTLOADER_ENTRY >> 8, FW_VERSION_BOOTLOADER_ENTRY & 0xFF);
    on_firmware_update_result_->trigger(false, "firmware predates bootloader entry");
    return;
  }

  uint8_t attempts = 0;
  update_attempts_pref_.load(&attempts);
  if (attempts >= max_update_attempts_) {
    ESP_LOGE(TAG, "update_firmware: max attempts (%u) already reached for target v%u, refusing",
             max_update_attempts_, firmware_fw_version_);
    on_firmware_update_result_->trigger(false, "max update attempts reached");
    return;
  }

  ESP_LOGI(TAG, "update_firmware: updating to v%u (attempt %u/%u)", firmware_fw_version_, attempts + 1,
           max_update_attempts_);

  s_update_in_progress = true;
  std::string error;
  bool ok = perform_firmware_update_(error);
  s_update_in_progress = false;

  if (ok) {
    ESP_LOGI(TAG, "update_firmware: success, now at v%u", firmware_fw_version_);
    uint8_t zero = 0;
    update_attempts_pref_.save(&zero);
    on_firmware_update_result_->trigger(true, "");
  } else {
    attempts++;
    update_attempts_pref_.save(&attempts);
    ESP_LOGE(TAG, "update_firmware: failed (attempt %u/%u): %s", attempts, max_update_attempts_, error.c_str());
    on_firmware_update_result_->trigger(false, error);
  }
}

// Port of the jig's reference sequence, see
// production_tools/programAndTest/src/fader_buddy_bootloader.cpp (updateFirmware()).
bool FaderBuddy::perform_firmware_update_(std::string &error_out) {
  App.feed_wdt();

  uint8_t ver;
  if (!bl_read_version_byte_(ver)) {
    error_out = "no I2C response";
    return false;
  }
  if (ver != BL_VERSION_MARKER) {
    if (!bl_enter_bootloader_()) {
      error_out = "enter bootloader cmd failed";
      return false;
    }
    if (!bl_wait_for_marker_(2000)) {
      error_out = "no bootloader marker";
      return false;
    }
  }

  uint8_t bl_ver, status, last_err;
  if (!bl_get_status_(bl_ver, status, last_err)) {
    error_out = "no bootloader status";
    return false;
  }

  ESP_LOGD(TAG, "update_firmware: erasing application section");
  if (!bl_erase_app_()) {
    error_out = "erase failed";
    return false;
  }

  uint32_t pages = firmware_image_length_ / BL_PAGE_SIZE;
  for (uint32_t p = 0; p < pages; p++) {
    uint16_t page_addr = BL_APP_START + (uint16_t) (p * BL_PAGE_SIZE);
    if (!bl_set_page_addr_(page_addr)) {
      error_out = "set page addr failed";
      return false;
    }
    for (uint8_t f = 0; f < BL_FRAMES_PER_PAGE; f++) {
      const uint8_t *chunk = firmware_image_ + (p * BL_PAGE_SIZE) + (f * BL_FRAME_DATA_LEN);
      if (!bl_send_frame_(chunk)) {
        error_out = "send frame failed";
        return false;
      }
    }
    if (p % 16 == 0) {
      App.feed_wdt();
      ESP_LOGD(TAG, "update_firmware: writing page %u/%u", (unsigned) (p + 1), (unsigned) pages);
    }
  }

  {
    uint8_t bv, st, le;
    if (bl_get_status_(bv, st, le) && le != BL_ERR_NONE) {
      error_out = "nvm err=" + std::to_string(le) + " after write";
      return false;
    }
  }

  App.feed_wdt();
  ESP_LOGD(TAG, "update_firmware: verifying");
  uint16_t crc;
  if (!bl_get_image_crc16_(BL_APP_START, (uint16_t) firmware_image_length_, crc)) {
    error_out = "crc read failed";
    return false;
  }
  if (crc != firmware_image_crc16_) {
    char buf[48];
    snprintf(buf, sizeof(buf), "CRC mismatch got=0x%04X exp=0x%04X", crc, firmware_image_crc16_);
    error_out = buf;
    return false;
  }

  if (!bl_run_app_()) {
    error_out = "run app cmd failed";
    return false;
  }
  uint8_t app_ver;
  if (!bl_wait_for_app_(2000, app_ver)) {
    error_out = "app did not start";
    return false;
  }

  uint16_t fw;
  if (!read_fw_version_(fw)) {
    error_out = "no fw version after update";
    return false;
  }
  if (fw != firmware_fw_version_) {
    error_out = "fw version mismatch after update";
    return false;
  }

  return true;
}

// --- Bootloader wire-protocol primitives ---

bool FaderBuddy::bl_write_retry_(const uint8_t *data, size_t len, uint8_t attempts, uint32_t retry_delay_ms) {
  for (uint8_t a = 0; a < attempts; a++) {
    if (this->write(data, len) == esphome::i2c::ErrorCode::NO_ERROR)
      return true;
    delay(retry_delay_ms);  // bootloader may be stalled in a flash erase/write
  }
  return false;
}

bool FaderBuddy::bl_read_retry_(const uint8_t *reg, size_t reg_len, uint8_t *out, size_t out_len, uint8_t attempts,
                                uint32_t retry_delay_ms) {
  for (uint8_t a = 0; a < attempts; a++) {
    if (this->write_read(reg, reg_len, out, out_len) == esphome::i2c::ErrorCode::NO_ERROR)
      return true;
    delay(retry_delay_ms);
  }
  return false;
}

bool FaderBuddy::bl_read_bare_retry_(uint8_t *out, size_t len, uint8_t attempts, uint32_t retry_delay_ms) {
  for (uint8_t a = 0; a < attempts; a++) {
    if (this->read(out, len) == esphome::i2c::ErrorCode::NO_ERROR)
      return true;
    delay(retry_delay_ms);  // target still busy (e.g. computing a CRC) -> NAK
  }
  return false;
}

bool FaderBuddy::bl_read_version_byte_(uint8_t &version) {
  uint8_t reg = REG_VERSION;
  return bl_read_retry_(&reg, 1, &version, 1, 1, 0);  // single-shot probe, no retry
}

bool FaderBuddy::bl_enter_bootloader_() {
  uint8_t buf[5] = {
      REG_ENTER_BOOTLOADER,
      (uint8_t) (ENTER_BOOTLOADER_MAGIC >> 24),
      (uint8_t) (ENTER_BOOTLOADER_MAGIC >> 16),
      (uint8_t) (ENTER_BOOTLOADER_MAGIC >> 8),
      (uint8_t) (ENTER_BOOTLOADER_MAGIC),
  };
  return bl_write_retry_(buf, sizeof(buf), 3, 5);
}

bool FaderBuddy::bl_wait_for_marker_(uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t v;
    if (bl_read_version_byte_(v) && v == BL_VERSION_MARKER)
      return true;
    delay(5);
  }
  return false;
}

bool FaderBuddy::bl_wait_for_app_(uint32_t timeout_ms, uint8_t &version) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t v;
    if (bl_read_version_byte_(v) && v != BL_VERSION_MARKER && v != 0xFF && v != 0x00) {
      version = v;
      return true;
    }
    delay(5);
  }
  return false;
}

bool FaderBuddy::bl_get_status_(uint8_t &bl_version, uint8_t &status, uint8_t &last_error) {
  uint8_t reg = BL_CMD_GET_STATUS;
  uint8_t out[3];
  if (!bl_read_retry_(&reg, 1, out, 3))
    return false;
  bl_version = out[0];
  status = out[1];
  last_error = out[2];
  return true;
}

bool FaderBuddy::bl_erase_app_() {
  uint8_t reg = BL_CMD_ERASE_APP;
  if (!bl_write_retry_(&reg, 1, 3, 5))
    return false;
  // The erase stalls the target CPU (~hundreds of ms); wait for it to answer,
  // then confirm no NVM error was recorded.
  uint8_t v, s, e;
  for (uint8_t a = 0; a < 200; a++) {
    if (bl_get_status_(v, s, e))
      return e == BL_ERR_NONE;
    delay(10);
  }
  return false;
}

bool FaderBuddy::bl_set_page_addr_(uint16_t addr) {
  uint8_t buf[3] = {BL_CMD_SET_PAGE_ADDR, (uint8_t) (addr >> 8), (uint8_t) (addr & 0xFF)};
  return bl_write_retry_(buf, sizeof(buf));
}

bool FaderBuddy::bl_send_frame_(const uint8_t *data16) {
  uint8_t buf[1 + BL_FRAME_DATA_LEN + 2];
  buf[0] = BL_CMD_SEND_FRAME;
  uint16_t crc = BL_CRC16_INIT;
  for (uint8_t i = 0; i < BL_FRAME_DATA_LEN; i++) {
    buf[1 + i] = data16[i];
    crc = bl_crc16_update(crc, data16[i]);
  }
  buf[1 + BL_FRAME_DATA_LEN] = (uint8_t) (crc >> 8);
  buf[1 + BL_FRAME_DATA_LEN + 1] = (uint8_t) (crc & 0xFF);
  return bl_write_retry_(buf, sizeof(buf));
}

bool FaderBuddy::bl_get_image_crc16_(uint16_t addr, uint16_t len, uint16_t &crc) {
  uint8_t buf[5] = {BL_CMD_GET_VERSION_CRC16, (uint8_t) (addr >> 8), (uint8_t) (addr & 0xFF), (uint8_t) (len >> 8),
                    (uint8_t) (len & 0xFF)};
  // The bootloader computes the CRC over the whole range after releasing the bus
  // (it can't clock-stretch the whole computation), then NAKs reads until done.
  if (!bl_write_retry_(buf, sizeof(buf), 3, 5))
    return false;
  delay(40);  // typical whole-image compute time; the read retries cover the rest
  uint8_t out[3];
  if (!bl_read_bare_retry_(out, 3, 80, 5))
    return false;
  crc = ((uint16_t) out[1] << 8) | out[2];
  return true;
}

bool FaderBuddy::bl_run_app_() {
  uint8_t reg = BL_CMD_RUN_APP;
  return bl_write_retry_(&reg, 1, 3, 5);
}

bool FaderBuddy::read_fw_version_(uint16_t &version) {
  uint8_t reg = REG_FW_VERSION;
  uint8_t out[2];
  if (!bl_read_retry_(&reg, 1, out, 2))
    return false;
  version = ((uint16_t) out[0] << 8) | out[1];
  return true;
}

}  // namespace fader_buddy
}  // namespace esphome
