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

#include "fader_buddy_host.h"

#include <stdio.h>

#include "bootloader_protocol.h"

namespace faderbuddy {

#define FB_LOGE(...) logf(platform_, LogLevel::ERROR, __VA_ARGS__)
#define FB_LOGW(...) logf(platform_, LogLevel::WARN, __VA_ARGS__)
#define FB_LOGI(...) logf(platform_, LogLevel::INFO, __VA_ARGS__)
#define FB_LOGCONFIG(...) logf(platform_, LogLevel::CONFIG, __VA_ARGS__)
#define FB_LOGD(...) logf(platform_, LogLevel::DEBUG, __VA_ARGS__)

// The boot-time version probe is the one read that decides whether this fader
// is usable at all, and it runs at the worst possible moment: the fader may
// still be coming out of reset, and the bus is settling behind however many
// other faders are chained on it. Retry it, and see probe_and_init_() for what
// happens when the retries still come up empty.
static const uint8_t SETUP_PROBE_ATTEMPTS = 5;
static const uint32_t SETUP_PROBE_RETRY_MS = 20;

// A fader that misses its setup probe is re-probed indefinitely, on a doubling
// backoff capped at 30s, so a late power ramp, a cable seated after boot or an
// intermittent connection recovers without a host reboot. A probe of an
// absent fader is just an address NAK, so this costs the other faders on the
// bus next to nothing.
static const uint32_t RETRY_PROBE_FIRST_MS = 1000;
static const uint32_t RETRY_PROBE_MAX_MS = 30000;

// How long an update waits for the user to let go of the fader.
static const uint32_t UPDATE_TOUCH_WAIT_MS = 5000;

FaderBuddy::FaderBuddy(Transport &transport, Platform &platform)
    : device_(transport, platform), platform_(platform), updater_(device_, this) {}

// --- Lifecycle ---

void FaderBuddy::begin() { probe_and_init_(true); }

void FaderBuddy::loop() {
  if (incompatible_) {
    return;
  }
  if (update_phase_ != UPDATE_PHASE_IDLE) {
    update_tick_();
    return;
  }
  if (poll_interval_ms_ != 0) {
    uint32_t now = platform_.millis();
    if (now - last_poll_ >= poll_interval_ms_) {
      last_poll_ = now;
      poll();
    }
  }
}

void FaderBuddy::poll() {
  if (incompatible_) {
    return;
  }
  // An update owns the bus and has the fader in its bootloader, where
  // REG_STATE means nothing. Skip the poll entirely until it finishes.
  if (update_phase_ != UPDATE_PHASE_IDLE) {
    return;
  }
  // Never got a usable answer. Nothing below can work, so spend the poll on
  // the backoff probe instead - the fader may yet turn up.
  if (awaiting_device_) {
    retry_probe_();
    return;
  }

  fire_deferred_moves_();

  if (!read_sensor_data_()) {
    FB_LOGW("Failed to read from sensor.");
  }
}

// Probe REG_VERSION and, if the answer is usable, finish initializing. Called
// at begin() and, while the fader is still unresponsive, again from poll() - so
// a fader that shows up late initializes itself instead of staying dead until
// the host reboots.
void FaderBuddy::probe_and_init_(bool first_attempt) {
  uint8_t probe = 0;
  // Retry hard once, at begin(), where a few tens of milliseconds buy a fader
  // that is merely slow to boot. The periodic re-probe is single-shot: it
  // already repeats on its backoff, and blocking for 100ms a time on an absent
  // fader would cost far more than it could win.
  uint8_t attempts = first_attempt ? SETUP_PROBE_ATTEMPTS : 1;
  if (!device_.read_probe(probe, attempts, SETUP_PROBE_RETRY_MS)) {
    if (!first_attempt) {
      FB_LOGD("Fader still not answering the version probe");
      return;
    }
    // Not fatal: the re-probing, and any firmware update that could recover a
    // wedged fader, both have to keep running.
    FB_LOGW("Init: no response from the fader after %d attempts. Will keep re-probing in the background.",
            SETUP_PROBE_ATTEMPTS);
    start_awaiting_device_();
    notify_status_changed_();
    return;
  }

  if (!first_attempt) {
    FB_LOGI("Fader is answering now; completing initialization");
  }
  awaiting_device_ = false;

  // A resident bootloader answers the version probe with its own marker
  // instead of a protocol version. Nothing else here works against it, but
  // noting it lets a firmware update still be installed to recover it.
  if (probe == BL_VERSION_MARKER) {
    bootloader_resident_ = true;
    FB_LOGW("Init: fader is sitting in its bootloader (no working app image). "
            "Install the firmware update to recover it.");
  }

  // Older protocols really are incompatible - the register layout differs -
  // but a NEWER one is not, because bumps are only made for changes a host can
  // ignore. Failing on those would brick this host on a fader that merely got
  // updated, so warn and carry on instead.
  //
  // There is no recovery path to offer here either: I2C bootloader entry
  // arrived in firmware 1.3, long after protocol v5, so anything reporting
  // less than v5 also predates REG_ENTER_BOOTLOADER and can only be migrated
  // over UPDI.
  if (probe < I2C_PROTOCOL_VERSION) {
    FB_LOGE("Init: Incompatible I2C protocol version. Expected at least %d but got %d. "
            "This firmware also predates I2C bootloader entry, so it needs a one-time "
            "UPDI reflash - no update over I2C is possible.",
            I2C_PROTOCOL_VERSION, probe);
    incompatible_ = true;
    notify_status_changed_();
    return;
  }
  if (probe > I2C_PROTOCOL_VERSION && !bootloader_resident_) {
    FB_LOGW("Fader reports protocol v%d, newer than the v%d this host library was built against. "
            "Continuing, but consider updating.",
            probe, I2C_PROTOCOL_VERSION);
  }

  FB_LOGCONFIG("FaderBuddy initialized (host library %s, protocol v%d)", FADER_BUDDY_HOSTLIB_VERSION, probe);

  read_serial_number_();
  read_firmware_version_();
  if (read_motor_calibration_() == MOTOR_CAL_ABSENT) {
    calibration_status_ = CALIBRATION_NEEDED;
    notify_status_changed_();
  }

  // Flag a config that asks for something this fader's firmware cannot do, at
  // startup rather than waiting for the first move to warn.
  if (!speed_supported_) {
    for (uint8_t i = 0; i < NUM_LAYERS; i++) {
      if (layer_states_[i].default_speed != LAYER_SPEED_FULL) {
        FB_LOGW("Layer %d sets default_speed, but this fader's firmware does not support move "
                "speed. Moves will run at full speed. Update the fader firmware to %d.%d or newer.",
                i, FW_VERSION_MOVE_SPEED >> 8, FW_VERSION_MOVE_SPEED & 0xFF);
        break;
      }
    }
  }

  for (uint8_t i = 0; i < NUM_LAYERS; i++) {
    if (initial_haptic_configs_[i].valid) {
      FB_LOGCONFIG("Sending initial haptic config for layer %d: mode=%d, detents=%d, strength=%d", i,
                   initial_haptic_configs_[i].mode, initial_haptic_configs_[i].detent_count,
                   initial_haptic_configs_[i].detent_strength);
      send_layer_haptic_config_(i, initial_haptic_configs_[i].mode, initial_haptic_configs_[i].detent_count,
                                initial_haptic_configs_[i].detent_strength);
    }
  }
  set_active_layer(0);
}

void FaderBuddy::start_awaiting_device_() {
  awaiting_device_ = true;
  retry_probe_backoff_ms_ = RETRY_PROBE_FIRST_MS;
  retry_probe_at_ = platform_.millis() + RETRY_PROBE_FIRST_MS;
}

// Re-probe an unresponsive fader on a doubling backoff. Driven from poll(),
// which runs far more often than a probe should - the clock, not the poll
// rate, sets the cadence.
void FaderBuddy::retry_probe_() {
  if ((int32_t) (platform_.millis() - retry_probe_at_) < 0) {
    return;
  }
  retry_probe_backoff_ms_ = retry_probe_backoff_ms_ * 2 < RETRY_PROBE_MAX_MS ? retry_probe_backoff_ms_ * 2
                                                                             : RETRY_PROBE_MAX_MS;
  retry_probe_at_ = platform_.millis() + retry_probe_backoff_ms_;
  probe_and_init_(false);
}

// The serial is a static factory ID, so this only needs to run once.
void FaderBuddy::read_serial_number_() {
  // REG_SERIAL is an app register, so there is nothing to read while the
  // bootloader is resident. It is read again once an update puts an app back.
  if (bootloader_resident_) {
    FB_LOGW("Serial number unavailable: fader is in its bootloader");
    return;
  }

  uint8_t serial[SERIAL_NUMBER_LEN];
  if (!device_.read_serial_number(serial)) {
    FB_LOGW("Failed to read serial number");
    return;
  }
  for (size_t i = 0; i < SERIAL_NUMBER_LEN; i++) {
    snprintf(serial_number_ + i * 2, 3, "%02X", serial[i]);
  }
  FB_LOGCONFIG("Serial number: %s", serial_number_);
  if (listener_ != nullptr) {
    listener_->on_serial_number(serial_number_);
  }
}

// Read the reported firmware version and derive what this fader can do.
// Firmware predating the register leaves the bus undriven, so the read
// succeeds and returns 0xFFFF rather than failing.
void FaderBuddy::read_firmware_version_() {
  // REG_FW_VERSION is an app register; don't read garbage from a bootloader
  // and report it.
  if (bootloader_resident_) {
    firmware_version_ = FW_VERSION_NONE;
    speed_supported_ = false;
    FB_LOGCONFIG("Fader status: in bootloader, no application installed");
    notify_status_changed_();
    return;
  }

  uint16_t version = FW_VERSION_NONE;
  if (!device_.read_fw_version(version)) {
    FB_LOGW("Failed to read firmware version; assuming pre-1.1 firmware");
    version = FW_VERSION_NONE;
  }
  firmware_version_ = normalize_fw_version(version);
  speed_supported_ = fw_supports_move_speed(firmware_version_);

  char buf[8];
  format_fw_version(firmware_version_, buf, sizeof(buf));
  FB_LOGCONFIG("Fader firmware: %s%s", buf, fw_version_known(firmware_version_) ? "" : " or older");
  notify_status_changed_();
}

// Log what self-calibration measured about this fader's motor, and the
// feedforward it derived. Diagnostic only, but these are the numbers to look
// at when a fader hunts or settles slowly.
FaderBuddy::MotorCalReadResult FaderBuddy::read_motor_calibration_() {
  if (!fw_supports_motor_cal(firmware_version_)) {
    return MOTOR_CAL_UNKNOWN;  // older firmware has no such register
  }
  MotorCal cal;
  if (!device_.read_motor_cal(cal)) {
    FB_LOGW("Failed to read motor calibration");
    return MOTOR_CAL_UNKNOWN;
  }
  if (!cal.valid) {
    FB_LOGCONFIG("Motor: not characterised, using the default plant model "
                 "(vel_min %u ADC/s, deadband %d). Run self-calibration to measure this unit.",
                 cal.vel_min, cal.deadband);
    return MOTOR_CAL_ABSENT;
  }
  FB_LOGCONFIG("Motor: breakaway %d/%d duty, k %d/%d ADC/s per duty, jump %u/%u ADC/s (rising/falling)",
               cal.breakaway_rising, cal.breakaway_falling, cal.k_rising, cal.k_falling, cal.v_jump_rising,
               cal.v_jump_falling);
  FB_LOGCONFIG("Motor: vel_min %u ADC/s, deadband %d ADC counts", cal.vel_min, cal.deadband);
  return MOTOR_CAL_PRESENT;
}

void FaderBuddy::fire_deferred_moves_() {
  for (uint8_t layer = 0; layer < NUM_LAYERS; layer++) {
    LayerState &ls = layer_states_[layer];
    if (ls.has_deferred_value && ls.value_change_min_interval > 0) {
      uint32_t now = platform_.millis();
      if (now - ls.last_trigger_time >= ls.value_change_min_interval) {
        FB_LOGI("Deferred movement to %03d (user) on layer %d", ls.deferred_value, layer);
        ls.last_trigger_time = now;
        ls.has_deferred_value = false;
        if (listener_ != nullptr) {
          listener_->on_manual_move(ls.deferred_value, layer);
        }
      }
    }
  }
}

bool FaderBuddy::read_sensor_data_() {
  FaderState s;
  if (!device_.read_state(s)) {
    FB_LOGE("Failed to read data");
    return false;
  }
  last_state_ = s;
  uint8_t active_layer = s.active_layer;

  // The fader's mode changes on its own - self-calibration finishing, a move
  // timing out into MODE_ERROR - and nothing else reports that.
  if (s.mode != last_mode_) {
    Mode previous = last_mode_;
    last_mode_ = s.mode;

    if (s.mode == MODE_SELF_CALIBRATION) {
      FB_LOGI("Self-calibration started");
      calibration_status_ = CALIBRATION_RUNNING;
      notify_status_changed_();
    } else if (previous == MODE_SELF_CALIBRATION) {
      if (s.mode == MODE_ERROR) {
        calibration_status_ = CALIBRATION_FAILED;
        FB_LOGE("Self-calibration failed: the endpoint sweep found no usable travel. "
                "Check the motor and potentiometer wiring. The fader ignores position "
                "commands until the error is cleared.");
      } else {
        // Re-read what it measured: the firmware keeps the endpoints but throws
        // the motor measurement away if any pass of it was implausible, so a run
        // can end normally and still leave the fader on default tuning.
        FB_LOGI("Self-calibration complete");
        if (read_motor_calibration_() == MOTOR_CAL_ABSENT) {
          FB_LOGW("Self-calibration found the endpoints, but the motor measurement was "
                  "implausible and was discarded. The fader is using default motor tuning.");
          calibration_status_ = CALIBRATION_MOTOR_DEFAULTS;
        } else {
          calibration_status_ = CALIBRATION_SUCCEEDED;
        }
      }
      notify_status_changed_();
    } else if (s.mode == MODE_ERROR) {
      FB_LOGW("Fader latched MODE_ERROR (a move did not reach its target). "
              "Position commands are ignored until the error is cleared.");
    } else {
      FB_LOGD("Mode %d -> %d", previous, s.mode);
    }
  }

  // Position changes, tracked per layer.
  LayerState &ls = layer_states_[active_layer];
  if (s.position != ls.last_hw_position || s.position_nonce != ls.last_position_nonce) {
    ls.last_hw_position = s.position;
    ls.last_position_nonce = s.position_nonce;

    uint8_t user_position = invert_ ? (uint8_t) (255 - s.position) : s.position;
    // Every change, regardless of mode or rate limiting.
    if (listener_ != nullptr) {
      listener_->on_raw_position_update(user_position, active_layer);
    }
    if (s.mode == MODE_INPUT_ACTIVE || s.mode == MODE_INPUT_IDLE) {
      ls.deferred_value = user_position;
      ls.has_deferred_value = true;
    }
  }

  if (ls.has_deferred_value) {
    uint32_t now = platform_.millis();
    if (now - ls.last_trigger_time >= ls.value_change_min_interval) {
      FB_LOGD("State: %08x -- Current position: %03d, position_nonce: %d, active_layer: %d, touch: %01d, "
              "mode: %d, adc: %d, double_tap_nonce: %d",
              (unsigned int) s.raw, s.position, s.position_nonce, active_layer, s.touch, s.mode, s.raw_adc,
              s.double_tap_nonce);
      FB_LOGI("Movement to %03d (user) on layer %d", ls.deferred_value, active_layer);
      ls.last_trigger_time = now;
      ls.has_deferred_value = false;
      if (listener_ != nullptr) {
        listener_->on_manual_move(ls.deferred_value, active_layer);
      }
    }
  }

  if (s.touch != last_touch_) {
    FB_LOGI("Touch changed to %s on layer %d", s.touch ? "true" : "false", active_layer);
    last_touch_ = s.touch;
    if (listener_ != nullptr) {
      listener_->on_touch_change(s.touch, active_layer);
    }
  }

  if (s.double_tap_nonce != last_double_tap_nonce_) {
    FB_LOGI("Double tap detected on layer %d -- %d", active_layer, s.double_tap_nonce);
    last_double_tap_nonce_ = s.double_tap_nonce;
    if (listener_ != nullptr) {
      listener_->on_double_tap(active_layer);
    }
  }
  return true;
}

// --- Layers and movement ---

bool FaderBuddy::write_with_retry_(const uint8_t *data, size_t len, uint8_t retries) {
  if (device_.write(data, len, retries, 1)) {
    return true;
  }
  FB_LOGE("I2C write failed after %d attempts", retries);
  return false;
}

void FaderBuddy::set_active_layer(uint8_t layer_index) {
  if (layer_index >= NUM_LAYERS) {
    FB_LOGE("Invalid layer index: %d", layer_index);
    return;
  }
  uint8_t buffer[] = {REG_ACTIVE_LAYER, layer_index};
  if (write_with_retry_(buffer, sizeof(buffer))) {
    FB_LOGI("Set active layer to %d", layer_index);
  } else {
    FB_LOGE("Failed to set active layer %d", layer_index);
  }
}

uint8_t FaderBuddy::get_active_layer() const {
  uint8_t layer = 0;
  if (!device_.read_active_layer(layer)) {
    logf(platform_, LogLevel::ERROR, "Failed to read active layer");
    return 0;
  }
  return layer;
}

void FaderBuddy::remote_move_to(uint8_t position, uint8_t layer) {
  if (layer >= NUM_LAYERS) {
    FB_LOGE("Invalid layer index: %d", layer);
    return;
  }
  remote_move_to(position, layer, layer_states_[layer].default_speed);
}

void FaderBuddy::remote_move_to(uint8_t position, uint8_t layer, uint8_t speed) {
  if (layer >= NUM_LAYERS) {
    FB_LOGE("Invalid layer index: %d", layer);
    return;
  }
  uint8_t hw_position = invert_ ? (uint8_t) (255 - position) : position;

  // Firmware without the speed byte drops a 4-byte write entirely, so fall
  // back to a full-speed move rather than letting the fader not move at all.
  if (speed != LAYER_SPEED_FULL && !speed_supported_) {
    if (!warned_speed_unsupported_) {
      warned_speed_unsupported_ = true;
      FB_LOGW("Requested move speed %d, but this fader's firmware does not support it "
              "(needs %d.%d or newer). Moving at full speed instead. This warning is logged once.",
              speed, FW_VERSION_MOVE_SPEED >> 8, FW_VERSION_MOVE_SPEED & 0xFF);
    }
    speed = LAYER_SPEED_FULL;
  }

  // Always send the speed when the firmware understands it, full speed
  // included. A 3-byte write leaves the layer's STORED speed alone, so after
  // any slower move it would silently re-apply that old limit. Firmware
  // predating the byte ignores a 4-byte write entirely, so that case (where
  // speed was forced to LAYER_SPEED_FULL above) still sends 3.
  uint8_t buffer[] = {REG_LAYER_TARGET, layer, hw_position, speed};
  size_t len = speed_supported_ ? 4 : 3;
  if (write_with_retry_(buffer, len)) {
    FB_LOGD("Set layer %d target to %d (user position) at speed %d", layer, position, speed);
  } else {
    FB_LOGE("Failed to write layer %d target", layer);
  }
}

uint8_t FaderBuddy::get_position(uint8_t layer) const {
  if (layer >= NUM_LAYERS) {
    logf(platform_, LogLevel::ERROR, "Invalid layer index: %d", layer);
    return 0;
  }
  uint8_t hw = 0;
  if (!device_.read_layer_target(layer, hw)) {
    logf(platform_, LogLevel::ERROR, "Failed to read layer position");
    return 0;
  }
  return invert_ ? (uint8_t) (255 - hw) : hw;
}

void FaderBuddy::send_layer_haptic_config_(uint8_t layer, uint8_t mode, uint8_t detent_count,
                                           uint8_t detent_strength) {
  uint16_t config = encode_haptic_config(mode, detent_count, detent_strength);
  uint8_t buffer[] = {REG_LAYER_HAPTIC_CONFIG, layer, (uint8_t) (config >> 8), (uint8_t) (config & 0xFF)};
  if (!write_with_retry_(buffer, sizeof(buffer))) {
    FB_LOGE("Failed to write layer haptic config for layer %d", layer);
  }
}

void FaderBuddy::store_initial_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count,
                                                   uint8_t detent_strength) {
  if (layer >= NUM_LAYERS) {
    FB_LOGE("Invalid layer index: %d", layer);
    return;
  }
  initial_haptic_configs_[layer].mode = mode;
  initial_haptic_configs_[layer].detent_count = detent_count;
  initial_haptic_configs_[layer].detent_strength = detent_strength;
  initial_haptic_configs_[layer].valid = true;
}

void FaderBuddy::set_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count,
                                         uint8_t detent_strength) {
  if (layer >= NUM_LAYERS) {
    FB_LOGE("Invalid layer index: %d", layer);
    return;
  }
  FB_LOGD("Set haptic config for layer %d: mode=%d, detents=%d, strength=%d", layer, mode, detent_count,
          detent_strength);
  send_layer_haptic_config_(layer, mode, detent_count, detent_strength);
}

void FaderBuddy::set_layer_value_change_min_interval(uint8_t layer, uint32_t min_interval_ms) {
  if (layer >= NUM_LAYERS) {
    FB_LOGE("Invalid layer index: %d", layer);
    return;
  }
  layer_states_[layer].value_change_min_interval = min_interval_ms;
}

void FaderBuddy::set_layer_default_speed(uint8_t layer, uint8_t speed) {
  if (layer >= NUM_LAYERS) {
    FB_LOGE("Invalid layer index: %d", layer);
    return;
  }
  layer_states_[layer].default_speed = speed;
}

uint8_t FaderBuddy::get_layer_default_speed(uint8_t layer) const {
  if (layer >= NUM_LAYERS) {
    return LAYER_SPEED_FULL;
  }
  return layer_states_[layer].default_speed;
}

void FaderBuddy::run_self_calibration() {
  uint8_t buffer = REG_SELF_CAL;
  if (!write_with_retry_(&buffer, 1)) {
    FB_LOGE("Failed to write self-calibration command");
  }
}

// --- Firmware update (ABOUT_I2C_BOOTLOADER.md) ---

void FaderBuddy::set_firmware_image(const uint8_t *image, uint32_t length, uint16_t image_crc16,
                                    uint16_t fw_version) {
  firmware_.data = image;
  firmware_.length = length;
  firmware_.crc16 = image_crc16;
  firmware_.fw_version = fw_version;
}

// Firmware older than FW_VERSION_BOOTLOADER_ENTRY ignores REG_ENTER_BOOTLOADER,
// and has no bootloader installed anyway (that takes a fuse write), so it can
// only be updated over UPDI.
bool FaderBuddy::bootloader_entry_supported() const {
  if (awaiting_device_ || bootloader_resident_) {
    // No version was read. Callers check for these states themselves; an
    // update is worth attempting in both.
    return false;
  }
  return fw_supports_bootloader_entry(firmware_version_);
}

bool FaderBuddy::firmware_update_offered() const {
  if (!has_firmware_image()) {
    return false;
  }
  if (awaiting_device_ || bootloader_resident_) {
    // No version to compare against, but installing an update is how a fader
    // that is wedged or stuck mid-flash gets recovered, so offer it. If the
    // fader is actually missing, the update's probe reports that.
    return true;
  }
  return firmware_version_ != firmware_.fw_version;
}

bool FaderBuddy::firmware_update_possible() const {
  if (!has_firmware_image()) {
    return false;
  }
  return awaiting_device_ || bootloader_resident_ || bootloader_entry_supported();
}

bool FaderBuddy::firmware_update_available() const {
  return firmware_update_possible() && firmware_update_offered();
}

// Install gets pressed speculatively, so handle the common no-op case from
// cached state, before taking the bus. force skips these checks.
void FaderBuddy::start_firmware_update(bool force) {
  if (!has_firmware_image()) {
    FB_LOGW("Firmware update: no firmware configured for this fader, ignoring request");
    notify_update_result_(false, "no firmware_image configured");
    return;
  }
  if (!force) {
    if (!firmware_update_possible()) {
      FB_LOGE("Firmware update: this fader's firmware predates I2C bootloader entry "
              "(needs >= v%u.%u); a one-time UPDI reflash is required - see "
              "ABOUT_I2C_BOOTLOADER.md. Ignoring request",
              FW_VERSION_BOOTLOADER_ENTRY >> 8, FW_VERSION_BOOTLOADER_ENTRY & 0xFF);
      notify_status_changed_();
      notify_update_result_(false, "UPDI reflash required");
      return;
    }
    if (!firmware_update_offered()) {
      FB_LOGI("Firmware update: already at v%u.%u, nothing to install. Ignoring request", firmware_version_ >> 8,
              firmware_version_ & 0xFF);
      notify_status_changed_();
      return;
    }
  }
  update_firmware();
}

void FaderBuddy::refresh_firmware_state() {
  if (firmware_update_in_progress()) {
    return;  // an update in progress publishes its own state
  }
  if (awaiting_device_) {
    probe_and_init_(false);
    if (awaiting_device_) {
      notify_status_changed_();  // still not responding; republish that
    }
    return;  // a successful probe_and_init_ publishes via read_firmware_version_()
  }
  read_firmware_version_();
}

bool FaderBuddy::reject_update_request_() {
  if (!has_firmware_image()) {
    FB_LOGE("update_firmware: no firmware_image configured");
    notify_update_result_(false, "no firmware_image configured");
    return true;
  }
  // An incompatible fader is never polled or looped, so an update started here
  // would never advance.
  if (incompatible_) {
    FB_LOGE("update_firmware: this fader failed to initialize; a one-time UPDI reflash is required - see "
            "ABOUT_I2C_BOOTLOADER.md");
    notify_status_changed_();
    notify_update_result_(false, "UPDI reflash required");
    return true;
  }
  if (update_phase_ != UPDATE_PHASE_IDLE) {
    FB_LOGW("update_firmware: this fader is already updating, skipping");
    notify_update_result_(false, "update already in progress");
    return true;
  }
  return false;
}

// Starting an update is just a state transition - the transfer runs a slice at
// a time from loop(). Everything here is decided up front, so a rejected
// request costs nothing and reports immediately.
void FaderBuddy::update_firmware() {
  if (reject_update_request_()) {
    return;
  }
  begin_update_();
}

void FaderBuddy::begin_update_() {
  update_progress_pct_ = 0xFF;
  // Don't interrupt the user: first wait (bounded) for the fader to go idle.
  update_phase_ = UPDATE_PHASE_WAIT_TOUCH_IDLE;
  update_deadline_ = platform_.millis() + UPDATE_TOUCH_WAIT_MS;
  FB_LOGD("update_firmware: waiting for fader");
  if (listener_ != nullptr) {
    listener_->on_firmware_update_stage("waiting for fader");
  }
}

void FaderBuddy::update_tick_() {
  switch (update_phase_) {
    case UPDATE_PHASE_IDLE:
      return;

    case UPDATE_PHASE_WAIT_TOUCH_IDLE: {
      read_sensor_data_();
      if (last_state_.mode == MODE_INPUT_ACTIVE) {
        if ((int32_t) (platform_.millis() - update_deadline_) >= 0) {
          finish_update_(false, "fader in use");
        }
        return;
      }
      FirmwareUpdater::Options options;
      // An "ensure up to date" automation must not see an error, or reflash.
      options.skip_if_current = true;
      update_phase_ = UPDATE_PHASE_RUNNING;
      updater_.start(firmware_, options);
      if (!updater_.running()) {
        finish_update_(false, updater_.error_message());
      }
      return;
    }

    case UPDATE_PHASE_RUNNING: {
      if (updater_.tick()) {
        return;
      }
      if (!updater_.succeeded()) {
        bootloader_resident_ = updater_.bootloader_resident();
        finish_update_(false, updater_.error_message());
        return;
      }
      if (updater_.was_noop()) {
        // Not a failure: the fader is where the config wants it.
        update_phase_ = UPDATE_PHASE_IDLE;
        read_firmware_version_();
        on_update_attempt_finished_(true, "");
        notify_update_result_(true, "");
        return;
      }
      finish_update_(true, "");
      return;
    }
  }
}

void FaderBuddy::on_update_stage(UpdateStage stage) {
  const char *label = nullptr;
  switch (stage) {
    case UpdateStage::WAIT_MARKER:
    case UpdateStage::ERASE:
    case UpdateStage::WRITE_CHECK:
    case UpdateStage::WAIT_APP:
      label = update_stage_name(stage);
      break;
    case UpdateStage::BODY_CHECK:
      // A one-page image skips the body verify.
      if (updater_.pages_total() > 1) {
        label = update_stage_name(stage);
      }
      break;
    default:
      break;
  }
  if (label == nullptr) {
    return;
  }
  FB_LOGD("update_firmware: %s", label);
  if (listener_ != nullptr) {
    listener_->on_firmware_update_stage(label);
  }
}

void FaderBuddy::on_update_pages_written(uint32_t written, uint32_t total) {
  if (updater_.stage() != UpdateStage::WRITE || total == 0) {
    return;  // page 0, written after the body has verified - no new percentage
  }
  // Page 0 is accounted for from the start, so the bar reaches 100% at the end
  // of the body, which is where the time goes.
  uint32_t pct = (written + 1) * 100 / total;
  publish_update_progress_("writing", (uint8_t) (pct > 100 ? 100 : pct));
}

// Coarse progress: steps of 5% keep the churn down.
void FaderBuddy::publish_update_progress_(const char *label, uint8_t pct) {
  if (pct == update_progress_pct_) {
    return;
  }
  if (update_progress_pct_ != 0xFF && pct < update_progress_pct_ + 5 && pct != 100) {
    return;
  }
  update_progress_pct_ = pct;
  FB_LOGD("update_firmware: %s %u%%", label, pct);
  if (listener_ != nullptr) {
    listener_->on_firmware_update_progress(label, pct);
  }
}

void FaderBuddy::finish_update_(bool ok, const char *error) {
  update_phase_ = UPDATE_PHASE_IDLE;

  if (ok) {
    FB_LOGI("update_firmware: success, now at v%u.%u", firmware_.fw_version >> 8, firmware_.fw_version & 0xFF);
    // Re-read rather than assume, so firmware_update_available() is right
    // without a reboot.
    bootloader_resident_ = false;
    read_firmware_version_();
    // A fader that was in its bootloader at startup never had its serial read
    // - REG_SERIAL is an app register. Now there's an app again.
    read_serial_number_();
    on_update_attempt_finished_(true, "");
    notify_update_result_(true, "");
    return;
  }

  // Copy: error may point into the updater, which the re-probe doesn't touch,
  // but the listener may start another update from its callback.
  char reason[64];
  snprintf(reason, sizeof(reason), "%s", error);
  FB_LOGE("update_firmware: failed: %s", reason);
  // Where the fader ended up depends on where it failed: still running the old
  // app, or stranded in the bootloader with the app erased. Re-probe rather
  // than guess.
  uint8_t probe;
  if (!device_.read_probe(probe, SETUP_PROBE_ATTEMPTS, SETUP_PROBE_RETRY_MS)) {
    // Still silent: report that, and go back to re-probing from poll().
    bootloader_resident_ = false;
    start_awaiting_device_();
    notify_status_changed_();
    on_update_attempt_finished_(false, reason);
    notify_update_result_(false, reason);
    return;
  }
  bootloader_resident_ = (probe == BL_VERSION_MARKER);
  read_firmware_version_();
  on_update_attempt_finished_(false, reason);
  notify_update_result_(false, reason);
}

void FaderBuddy::notify_status_changed_() {
  if (listener_ != nullptr) {
    listener_->on_status_changed();
  }
}

void FaderBuddy::notify_update_result_(bool ok, const char *error) {
  if (listener_ != nullptr) {
    listener_->on_firmware_update_result(ok, error);
  }
}

}  // namespace faderbuddy
