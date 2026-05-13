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

#include <Arduino.h>
#include <Wire.h>
#include "i2c_data.h"

#define MOTOR_FADER_I2C_ADDR 0x20  // Motor fader I2C base address (see firmware/src/main.cpp)

class MotorFaderI2C {
public:
  MotorFaderI2C(uint8_t address = MOTOR_FADER_I2C_ADDR);

  // Initialization
  void begin(TwoWire* wire);

  // Protocol functions
  bool readState(uint32_t& state);
  bool readProtocolVersion(uint8_t& version);
  bool writeTargetPosition(uint8_t target);  // Writes to layer 0
  bool readUptime(uint32_t& uptime);
  bool readTouchRaw(uint16_t& touchRaw);
  bool readSerialNumber(uint8_t serial[10]);  // Read 10-byte serial number
  bool readTouchDelta(int16_t& delta);
  bool readTouchReference(uint16_t& reference);
  bool readTouchRecalCount(uint16_t& recalCount);
  bool calibrateTouch();
  bool clearError();
  bool selfCalibrate();

private:
  uint8_t _address;
  TwoWire* _wire;

  bool readRegister(uint8_t reg, uint8_t* data, size_t len);
  bool writeRegister(uint8_t reg);
  bool writeRegister(uint8_t reg, uint8_t data);
};
