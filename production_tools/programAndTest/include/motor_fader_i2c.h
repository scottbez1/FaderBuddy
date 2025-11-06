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
  bool readTargetPosition(uint8_t& target);
  bool writeTargetPosition(uint8_t target);
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
