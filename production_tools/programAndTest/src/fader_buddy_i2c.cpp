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

#include "fader_buddy_i2c.h"

FaderBuddyI2C::FaderBuddyI2C(uint8_t address) : _address(address), _wire(nullptr) {}

void FaderBuddyI2C::begin(TwoWire* wire) {
  _wire = wire;
}

bool FaderBuddyI2C::readRegister(uint8_t reg, uint8_t* data, size_t len) {
  if (_wire == nullptr) {
    return false;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  if (_wire->requestFrom(_address, len) != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    data[i] = _wire->read();
  }
  return true;
}

bool FaderBuddyI2C::writeRegister(uint8_t reg) {
  if (_wire == nullptr) {
    return false;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  return _wire->endTransmission() == 0;
}

bool FaderBuddyI2C::writeRegister(uint8_t reg, uint8_t data) {
  if (_wire == nullptr) {
    return false;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(data);
  return _wire->endTransmission() == 0;
}

bool FaderBuddyI2C::readState(uint32_t& state) {
  uint8_t data[4];
  if (!readRegister(REG_STATE, data, 4)) {
    return false;
  }
  state = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
          ((uint32_t)data[2] << 8) | data[3];
  return true;
}

bool FaderBuddyI2C::readProtocolVersion(uint8_t& version) {
  return readRegister(REG_VERSION, &version, 1);
}

bool FaderBuddyI2C::writeTargetPosition(uint8_t target) {
  if (_wire == nullptr) {
    return false;
  }

  // Layer-addressed write to layer 0: [register, layer, position]
  _wire->beginTransmission(_address);
  _wire->write(REG_LAYER_TARGET);
  _wire->write(0);  // Layer 0
  _wire->write(target);
  return _wire->endTransmission() == 0;
}

bool FaderBuddyI2C::readUptime(uint32_t& uptime) {
  uint8_t data[4];
  if (!readRegister(REG_UPTIME, data, 4)) {
    return false;
  }
  uptime = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | data[3];
  return true;
}

bool FaderBuddyI2C::readTouchRaw(uint16_t& touchRaw) {
  uint8_t data[2];
  if (!readRegister(REG_TOUCH_RAW, data, 2)) {
    return false;
  }
  touchRaw = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

bool FaderBuddyI2C::readSerialNumber(uint8_t serial[10]) {
  return readRegister(REG_SERIAL, serial, 10);
}

bool FaderBuddyI2C::calibrateTouch() {
  return writeRegister(REG_CAL_TOUCH);
}

bool FaderBuddyI2C::clearError() {
  return writeRegister(REG_CLEAR_ERROR);
}

bool FaderBuddyI2C::selfCalibrate() {
  return writeRegister(REG_SELF_CAL);
}

bool FaderBuddyI2C::readTouchDelta(int16_t& delta) {
  uint8_t data[2];
  if (!readRegister(REG_TOUCH_DELTA, data, 2)) {
    return false;
  }
  // Combine bytes as signed 16-bit value
  delta = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  return true;
}

bool FaderBuddyI2C::readTouchReference(uint16_t& reference) {
  uint8_t data[2];
  if (!readRegister(REG_TOUCH_REF, data, 2)) {
    return false;
  }
  reference = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

bool FaderBuddyI2C::readTouchRecalCount(uint16_t& recalCount) {
  uint8_t data[2];
  if (!readRegister(REG_TOUCH_RECAL, data, 2)) {
    return false;
  }
  recalCount = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

