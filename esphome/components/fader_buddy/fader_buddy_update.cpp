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

#include "fader_buddy_update.h"

#ifdef USE_UPDATE

#include "fader_buddy.h"

namespace esphome {
namespace fader_buddy {

void FaderBuddyUpdate::perform(bool force) { this->parent_->start_firmware_update(force); }

void FaderBuddyUpdate::check() { this->parent_->refresh_firmware_state(); }

void FaderBuddyUpdate::publish_versions(const std::string &current_version, bool available) {
  this->update_info_.current_version = current_version;
  this->update_info_.has_progress = false;
  this->state_ = available ? update::UPDATE_STATE_AVAILABLE : update::UPDATE_STATE_NO_UPDATE;
  this->publish_state();
}

void FaderBuddyUpdate::publish_installing() {
  this->state_ = update::UPDATE_STATE_INSTALLING;
  this->publish_state();
}

void FaderBuddyUpdate::publish_pending() {
  this->state_ = update::UPDATE_STATE_INSTALLING;
  this->update_info_.has_progress = false;
  this->publish_state();
}

void FaderBuddyUpdate::publish_progress(uint8_t pct) {
  this->state_ = update::UPDATE_STATE_INSTALLING;
  this->update_info_.has_progress = true;
  this->update_info_.progress = pct;
  this->publish_state();
}

}  // namespace fader_buddy
}  // namespace esphome

#endif  // USE_UPDATE
