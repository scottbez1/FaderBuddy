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

#pragma once

#include "esphome/core/defines.h"

// USE_UPDATE is only defined when an update entity exists, so none of this is
// compiled in unless some fader has a firmware image configured.
#ifdef USE_UPDATE

#include <string>

#include "esphome/components/update/update_entity.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace fader_buddy {

class FaderBuddy;

// Home Assistant firmware update entity for one fader.
//
// Home Assistant decides whether an update is available by comparing the
// current and latest version strings. The API has no "available" flag and no
// error message field (see UpdateStateResponse in api_connection.cpp);
// UPDATE_STATE_AVAILABLE is only used on the device, by the update.is_available
// condition. Anything the user needs to see has to go in the version strings
// or the hub's status text sensor.
//
// This isn't a Component; the hub drives all of its state changes.
class FaderBuddyUpdate : public update::UpdateEntity, public Parented<FaderBuddy> {
 public:
  // HA's install button and the update.perform action. force skips the check
  // for whether there's anything to install, which allows a downgrade.
  void perform(bool force) override;
  // HA's "check for updates". Re-reads REG_FW_VERSION (or re-probes a fader
  // that never answered), so a fader reflashed over UPDI is picked up without
  // a reboot.
  void check() override;

  void set_title(const std::string &title) { this->update_info_.title = title; }
  void set_summary(const std::string &summary) { this->update_info_.summary = summary; }
  void set_release_url(const std::string &url) { this->update_info_.release_url = url; }
  void set_latest_version(const std::string &version) { this->update_info_.latest_version = version; }

  // Not installing: the fader's current version, and whether the packaged
  // image differs from it.
  void publish_versions(const std::string &current_version, bool available);
  // Installing, with no percentage (entering the bootloader, erasing,
  // verifying). Leaves any percentage already shown in place.
  void publish_installing();
  // Waiting behind another fader's update. Home Assistant shows this as
  // installing with no percentage. It has no install timeout - the ESPHome
  // integration's install call returns as soon as the command is sent, and
  // in_progress then tracks only what the device reports - so a long wait in
  // this state needs no heartbeat.
  void publish_pending();
  // Installing, writing pages: percentage, 0-100.
  void publish_progress(uint8_t pct);
};

}  // namespace fader_buddy
}  // namespace esphome

#endif  // USE_UPDATE
