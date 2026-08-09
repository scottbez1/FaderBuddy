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
