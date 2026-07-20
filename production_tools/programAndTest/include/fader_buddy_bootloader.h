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
 * I2C bootloader client for the ESP32 production/test jig.
 *
 * Drives a FaderBuddy's I2C bootloader (firmware/src/bootloader/) to stream a
 * new application image and verify it. Mirrors the update sequence the ESPHome
 * host will use (see ABOUT_I2C_BOOTLOADER.md sections 6 and 11).
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "i2c_data.h"
#include "bootloader_protocol.h"

class FaderBuddyBootloader {
public:
  // Progress callback: (stage description, current, total). current/total==0 for
  // non-quantified stages. Optional.
  typedef void (*ProgressFn)(const char* stage, uint32_t current, uint32_t total);

  FaderBuddyBootloader(uint8_t address = BL_I2C_BASE_ADDRESS);
  void begin(TwoWire* wire);

  // --- Low-level bootloader operations ---
  bool readVersionByte(uint8_t& version);                 // register 0x00
  bool enterBootloader();                                 // REG_ENTER_BOOTLOADER + magic
  bool waitForMarker(uint32_t timeout_ms);                // poll 0x00 until BL_VERSION_MARKER
  bool waitForApp(uint32_t timeout_ms, uint8_t& version); // poll 0x00 until a valid version
  bool getStatus(uint8_t& blVersion, uint8_t& status, uint8_t& lastError);
  bool eraseApp();
  bool setPageAddr(uint16_t addr);
  bool sendFrame(const uint8_t* data16);                  // 16 data bytes (+CRC added here)
  bool getImageCrc16(uint16_t addr, uint16_t len, uint16_t& crc);
  bool runApp();
  bool readFwVersion(uint16_t& version);                  // application REG_FW_VERSION

  // --- High-level full update ---
  // image points at the exact APPCODE bytes starting at flashStart (page-aligned
  // length). Returns true on success; on failure writes a short reason to err.
  bool updateFirmware(const uint8_t* image, uint32_t length, uint16_t flashStart,
                      uint16_t expectedImageCrc, uint16_t expectedFwVersion,
                      char* err, size_t errLen, ProgressFn progress = nullptr);

private:
  uint8_t _address;
  TwoWire* _wire;

  // Write with retry to ride out the bootloader's flash-write/erase CPU stalls.
  bool writeBytesRetry(const uint8_t* buf, size_t n, uint8_t attempts = 40,
                       uint32_t retryDelayMs = 5);
  // Register-style read: write regbuf, repeated-start, read rn bytes (with retry).
  bool readRetry(const uint8_t* regbuf, size_t regn, uint8_t* out, size_t rn,
                 uint8_t attempts = 40, uint32_t retryDelayMs = 5);
  // Bare read (no preceding register write) of rn bytes, with retry. Used to
  // poll a result the target computes after releasing the bus (e.g. the CRC).
  bool readBytesRetry(uint8_t* out, size_t rn, uint8_t attempts = 40,
                      uint32_t retryDelayMs = 5);
};
