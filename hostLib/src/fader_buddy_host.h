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

/*
 * One fader, as a host application sees it: layers, moves, haptics, touch and
 * gesture events, self-calibration, and firmware updates. This is the ESPHome
 * component's behaviour with the ESPHome removed - the component is a thin
 * wrapper over this class, and its public interface mirrors this one.
 *
 * Lifecycle:
 *   begin()  once. Probes the fader and initializes it. A fader that doesn't
 *            answer yet is re-probed on a backoff from poll().
 *   poll()   at the rate you want position/touch events (ESPHome: update()).
 *   loop()   as often as possible. Runs firmware updates a slice at a time,
 *            and calls poll() itself if set_poll_interval() was given.
 *
 * Events arrive through a FaderBuddyListener.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "fader_buddy_device.h"
#include "fader_buddy_platform.h"
#include "fader_buddy_protocol.h"
#include "fader_buddy_updater.h"
#include "i2c_data.h"

#define FADER_BUDDY_HOSTLIB_VERSION "0.1.0"

namespace faderbuddy {

class FaderBuddyListener {
 public:
  virtual ~FaderBuddyListener() = default;

  // The user moved the fader. Rate-limited per layer by
  // set_layer_value_change_min_interval(); the latest position always arrives.
  // Positions are user-facing (inverted if set_invert(true)).
  virtual void on_manual_move(uint8_t position, uint8_t layer) {
    (void) position;
    (void) layer;
  }
  // Every position change, local or remote, unthrottled.
  virtual void on_raw_position_update(uint8_t position, uint8_t layer) {
    (void) position;
    (void) layer;
  }
  virtual void on_touch_change(bool touched, uint8_t layer) {
    (void) touched;
    (void) layer;
  }
  virtual void on_double_tap(uint8_t layer) { (void) layer; }

  // Read once at begin() (and again after an update installs an app on a fader
  // that had none). Uppercase hex.
  virtual void on_serial_number(const char *serial) { (void) serial; }
  // Anything a status display would show changed: firmware version, device
  // reachability, calibration status, whether an update is offered.
  virtual void on_status_changed() {}

  // Firmware update in progress, with no percentage: "waiting for fader",
  // "entering bootloader", "erasing", "verifying", "starting app".
  virtual void on_firmware_update_stage(const char *label) { (void) label; }
  // Firmware update in progress, writing pages. Steps of at least 5%.
  virtual void on_firmware_update_progress(const char *label, uint8_t percent) {
    (void) label;
    (void) percent;
  }
  // Every update_firmware() call ends here exactly once, rejected requests
  // included. on_status_changed() has already fired with the fader's state
  // after the attempt.
  virtual void on_firmware_update_result(bool success, const char *error) {
    (void) success;
    (void) error;
  }
};

class FaderBuddy : protected UpdateObserver {
 public:
  enum CalibrationStatus : uint8_t {
    CALIBRATION_UNKNOWN,         // nothing worth showing
    CALIBRATION_NEEDED,          // startup: motor never characterised
    CALIBRATION_RUNNING,
    CALIBRATION_SUCCEEDED,
    CALIBRATION_MOTOR_DEFAULTS,  // endpoints found, motor measurement discarded
    CALIBRATION_FAILED,          // endpoint sweep found no usable travel
  };

  FaderBuddy(Transport &transport, Platform &platform);
  virtual ~FaderBuddy() = default;

  void set_listener(FaderBuddyListener *listener) { listener_ = listener; }

  // --- Lifecycle ---
  void begin();
  void poll();
  void loop();
  // If non-zero, loop() calls poll() at this interval. 0 (the default) leaves
  // calling poll() to you.
  void set_poll_interval(uint32_t interval_ms) { poll_interval_ms_ = interval_ms; }

  // --- Layers and movement ---
  void set_active_layer(uint8_t layer_index);
  uint8_t get_active_layer() const;
  // Moves at the layer's configured default speed (see set_layer_default_speed).
  void remote_move_to(uint8_t position, uint8_t layer = 0);
  // speed: unitless 0-255, where LAYER_SPEED_FULL (255) is full speed and 0
  // is the slowest the fader moves smoothly. It caps speed rather than
  // scheduling the move, so a shorter move takes proportionally less time.
  // Firmware too old for the speed byte moves at full speed (warned once).
  void remote_move_to(uint8_t position, uint8_t layer, uint8_t speed);
  uint8_t get_position(uint8_t layer = 0) const;
  void set_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count = 0, uint8_t detent_strength = 0);
  // Remembered and sent at begin(), before the fader is known to be there.
  void store_initial_layer_haptic_config(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);
  void run_self_calibration();
  void set_invert(bool invert) { invert_ = invert; }
  void set_layer_value_change_min_interval(uint8_t layer, uint32_t min_interval_ms);
  // Speed used for moves on this layer when the caller doesn't name one.
  // Tracked host-side and sent with each move, so it costs no extra traffic.
  void set_layer_default_speed(uint8_t layer, uint8_t speed);
  uint8_t get_layer_default_speed(uint8_t layer) const;

  // --- Identity and status ---
  // Packed (major << 8) | minor; FW_VERSION_NONE for firmware with no version
  // register (1.0 or older), or when no app is running.
  uint16_t get_firmware_version() const { return firmware_version_; }
  // Uppercase hex, "" until read.
  const char *get_serial_number() const { return serial_number_; }
  CalibrationStatus get_calibration_status() const { return calibration_status_; }
  // The last REG_STATE read, decoded.
  const FaderState &get_last_state() const { return last_state_; }
  // The fader didn't answer its probe; poll() keeps re-probing.
  bool is_awaiting_device() const { return awaiting_device_; }
  // The fader is in its bootloader with no working app.
  bool is_bootloader_resident() const { return bootloader_resident_; }
  // The fader speaks a protocol older than this library's - nothing works, and
  // it can only be fixed by a UPDI reflash. poll() and loop() do nothing.
  bool is_incompatible() const { return incompatible_; }
  bool speed_supported() const { return speed_supported_; }
  // Can this fader enter its bootloader over I2C? False for firmware older
  // than FW_VERSION_BOOTLOADER_ENTRY, and when no version could be read.
  bool bootloader_entry_supported() const;

  // --- Firmware update ---
  // image/length/crc16 describe the exact page-aligned APPCODE bytes;
  // fw_version is what the image reports once running.
  void set_firmware_image(const uint8_t *image, uint32_t length, uint16_t image_crc16, uint16_t fw_version);
  bool has_firmware_image() const { return firmware_.data != nullptr; }
  uint16_t get_packaged_firmware_version() const { return firmware_.fw_version; }
  uint32_t get_packaged_firmware_length() const { return firmware_.length; }
  // Start an update. Returns at once; the transfer runs from loop(), and the
  // outcome arrives at on_firmware_update_result(). A no-op success if the
  // fader already runs the packaged version. Waits (bounded) for the user to
  // let go of the fader before starting.
  virtual void update_firmware();
  // update_firmware(), but reject a request with nothing to install without
  // touching the bus. force skips that check, e.g. to downgrade.
  void start_firmware_update(bool force = false);
  virtual bool firmware_update_in_progress() const { return update_phase_ != UPDATE_PHASE_IDLE; }
  // Whether an update would do anything: offered && possible. Bus-free, from
  // cached state; update_firmware() re-checks over the wire.
  bool firmware_update_available() const;
  // Would installing the packaged image change anything? Deliberately ignores
  // whether it can be installed, so a fader needing a UPDI reflash still shows
  // as out of date.
  bool firmware_update_offered() const;
  // Could an install run against this fader at all?
  bool firmware_update_possible() const;
  // Re-read the fader's firmware version (or re-probe one that never answered).
  void refresh_firmware_state();

  // --- Lower layers ---
  const FaderBuddyDevice &device() const { return device_; }
  Platform &platform() const { return platform_; }

 protected:
  // Validate an update request, reporting a rejection to the listener.
  // Returns true if it was rejected.
  bool reject_update_request_();
  // Start an update that has passed reject_update_request_().
  void begin_update_();
  // An update started by begin_update_() has finished, successfully or not.
  // Called after the fader's state has been re-read and on_status_changed()
  // fired, just before on_firmware_update_result(). Rejected requests, which
  // never started, don't come through here.
  virtual void on_update_attempt_finished_(bool success, const char *error) {
    (void) success;
    (void) error;
  }

  // UpdateObserver
  void on_update_stage(UpdateStage stage) override;
  void on_update_pages_written(uint32_t written, uint32_t total) override;

 private:
  void probe_and_init_(bool first_attempt);
  void start_awaiting_device_();
  void retry_probe_();
  void read_serial_number_();
  void read_firmware_version_();
  enum MotorCalReadResult { MOTOR_CAL_UNKNOWN, MOTOR_CAL_ABSENT, MOTOR_CAL_PRESENT };
  MotorCalReadResult read_motor_calibration_();
  bool read_sensor_data_();
  bool write_with_retry_(const uint8_t *data, size_t len, uint8_t retries = 3);
  void send_layer_haptic_config_(uint8_t layer, uint8_t mode, uint8_t detent_count, uint8_t detent_strength);
  void fire_deferred_moves_();
  void update_tick_();
  void finish_update_(bool ok, const char *error);
  void publish_update_progress_(const char *label, uint8_t pct);
  void notify_status_changed_();
  void notify_update_result_(bool ok, const char *error);

  FaderBuddyDevice device_;
  Platform &platform_;
  FaderBuddyListener *listener_{nullptr};
  FirmwareUpdater updater_;

  uint32_t poll_interval_ms_{0};
  uint32_t last_poll_{0};

  FaderState last_state_{};
  // Faders normally come up idle, so this doesn't report a phantom transition
  // at startup - but one that boots into MODE_ERROR does get reported.
  Mode last_mode_{MODE_INPUT_IDLE};
  char serial_number_[SERIAL_NUMBER_LEN * 2 + 1] = {0};
  CalibrationStatus calibration_status_{CALIBRATION_UNKNOWN};
  bool invert_{false};
  bool last_touch_{false};
  uint8_t last_double_tap_nonce_{0};
  uint16_t firmware_version_{FW_VERSION_NONE};
  bool bootloader_resident_{false};
  bool awaiting_device_{false};
  bool incompatible_{false};
  uint32_t retry_probe_at_{0};
  uint32_t retry_probe_backoff_ms_{0};
  bool speed_supported_{false};
  bool warned_speed_unsupported_{false};

  struct LayerState {
    uint32_t value_change_min_interval{0};  // 0 = no rate limiting
    uint32_t last_trigger_time{0};
    uint8_t deferred_value{0};  // user-facing position for a deferred event
    bool has_deferred_value{false};
    uint8_t last_hw_position{0};
    uint8_t last_position_nonce{0};
    uint8_t default_speed{LAYER_SPEED_FULL};
  };
  LayerState layer_states_[NUM_LAYERS] = {};

  struct InitialHapticConfig {
    uint8_t mode;
    uint8_t detent_count;
    uint8_t detent_strength;
    bool valid;
  };
  InitialHapticConfig initial_haptic_configs_[NUM_LAYERS] = {};

  FirmwareImage firmware_{nullptr, 0, 0, 0};

  enum UpdatePhase : uint8_t {
    UPDATE_PHASE_IDLE = 0,
    UPDATE_PHASE_WAIT_TOUCH_IDLE,  // bounded wait for the user to let go
    UPDATE_PHASE_RUNNING,          // the FirmwareUpdater has the bus
  };
  UpdatePhase update_phase_{UPDATE_PHASE_IDLE};
  uint32_t update_deadline_{0};
  uint8_t update_progress_pct_{0xFF};  // last published step; 0xFF = none yet
};

}  // namespace faderbuddy
