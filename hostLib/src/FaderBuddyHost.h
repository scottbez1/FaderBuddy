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

// Everything, for applications. Layers, from the top:
//   fader_buddy_host.h      FaderBuddy - one fader as an application sees it
//   fader_buddy_updater.h   FirmwareUpdater - the I2C bootloader install sequence
//   fader_buddy_device.h    FaderBuddyDevice - typed register access, no policy
//   fader_buddy_protocol.h  wire-format codecs and firmware-version rules
//   fader_buddy_platform.h  Transport and Platform, what a host provides
//   fader_buddy_arduino.h   Transport/Platform for Arduino (TwoWire)

#pragma once

#include "fader_buddy_platform.h"
#include "fader_buddy_protocol.h"
#include "fader_buddy_device.h"
#include "fader_buddy_updater.h"
#include "fader_buddy_host.h"
#include "fader_buddy_arduino.h"
