#include "motor_fader_i2c.h"

MotorFaderI2C::MotorFaderI2C(uint8_t address) : _address(address), _wire(nullptr) {}

void MotorFaderI2C::begin(TwoWire* wire) {
  _wire = wire;
}

bool MotorFaderI2C::readRegister(uint8_t reg, uint8_t* data, size_t len) {
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

bool MotorFaderI2C::writeRegister(uint8_t reg) {
  if (_wire == nullptr) {
    return false;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  return _wire->endTransmission() == 0;
}

bool MotorFaderI2C::writeRegister(uint8_t reg, uint8_t data) {
  if (_wire == nullptr) {
    return false;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(data);
  return _wire->endTransmission() == 0;
}

bool MotorFaderI2C::readState(uint32_t& state) {
  uint8_t data[4];
  if (!readRegister(REG_STATE, data, 4)) {
    return false;
  }
  state = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
          ((uint32_t)data[2] << 8) | data[3];
  return true;
}

bool MotorFaderI2C::readProtocolVersion(uint8_t& version) {
  return readRegister(REG_VERSION, &version, 1);
}

bool MotorFaderI2C::readTargetPosition(uint8_t& target) {
  return readRegister(REG_TARGET, &target, 1);
}

bool MotorFaderI2C::writeTargetPosition(uint8_t target) {
  return writeRegister(REG_TARGET, target);
}

bool MotorFaderI2C::readUptime(uint32_t& uptime) {
  uint8_t data[4];
  if (!readRegister(REG_UPTIME, data, 4)) {
    return false;
  }
  uptime = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | data[3];
  return true;
}

bool MotorFaderI2C::readTouchRaw(uint16_t& touchRaw) {
  uint8_t data[2];
  if (!readRegister(REG_TOUCH_RAW, data, 2)) {
    return false;
  }
  touchRaw = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

bool MotorFaderI2C::calibrateTouch() {
  return writeRegister(REG_CAL_TOUCH);
}

bool MotorFaderI2C::clearError() {
  return writeRegister(REG_CLEAR_ERROR);
}

bool MotorFaderI2C::selfCalibrate() {
  return writeRegister(REG_SELF_CAL);
}
