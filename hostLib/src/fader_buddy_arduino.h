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
 * Arduino adapters: a Transport over TwoWire, and a Platform over millis(),
 * delay() and an optional Print for logging. Header-only, and compiled to
 * nothing outside Arduino, so the library still builds under ESP-IDF.
 */

#pragma once

#ifdef ARDUINO

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include "fader_buddy_platform.h"

namespace faderbuddy {

class WireTransport : public Transport {
 public:
  explicit WireTransport(TwoWire &wire, uint8_t address = 0x20) : wire_(wire), address_(address) {}

  void set_address(uint8_t address) { address_ = address; }
  uint8_t address() const { return address_; }

  bool write(const uint8_t *data, size_t len) override {
    wire_.beginTransmission(address_);
    wire_.write(data, len);
    return wire_.endTransmission() == 0;
  }

  bool read(uint8_t *data, size_t len) override {
    if (wire_.requestFrom(address_, len) != len) {
      return false;
    }
    for (size_t i = 0; i < len; i++) {
      data[i] = wire_.read();
    }
    return true;
  }

  bool write_read(const uint8_t *write_data, size_t write_len, uint8_t *read_data, size_t read_len) override {
    wire_.beginTransmission(address_);
    wire_.write(write_data, write_len);
    if (wire_.endTransmission(false) != 0) {  // repeated start
      return false;
    }
    return read(read_data, read_len);
  }

 private:
  TwoWire &wire_;
  uint8_t address_;
};

class ArduinoPlatform : public Platform {
 public:
  // log_output: where log lines go (e.g. &Serial), or nullptr for none.
  explicit ArduinoPlatform(Print *log_output = nullptr, LogLevel max_level = LogLevel::INFO)
      : log_output_(log_output), max_level_(max_level) {}

  uint32_t millis() override { return ::millis(); }
  void delay_ms(uint32_t ms) override { ::delay(ms); }

  void log(LogLevel level, const char *format, va_list args) override {
    if (log_output_ == nullptr || level > max_level_) {
      return;
    }
    static const char PREFIX[] = {'E', 'W', 'I', 'C', 'D', 'V'};
    // One write per line, so a line can't be split by output from elsewhere.
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "[fader_buddy %c] ", PREFIX[(uint8_t) level]);
    vsnprintf(buf + n, sizeof(buf) - n - 2, format, args);
    size_t len = strlen(buf);
    buf[len++] = '\r';
    buf[len++] = '\n';
    log_output_->write((const uint8_t *) buf, len);
  }

 private:
  Print *log_output_;
  LogLevel max_level_;
};

}  // namespace faderbuddy

#endif  // ARDUINO
