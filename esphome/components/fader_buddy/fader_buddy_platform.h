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
 * What the host library needs from whatever it runs on: a way to talk to one
 * fader, and a clock. Kept free of any framework header so the same code builds
 * under Arduino, ESP-IDF (ESPHome) and the native unit tests.
 *
 * Adapters: fader_buddy_arduino.h (TwoWire), and the ESPHome component's own,
 * which wraps its i2c::I2CDevice.
 */

#pragma once

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

namespace faderbuddy {

// One fader at one I2C address. Each call is a complete transaction and
// returns true only if every byte was acknowledged. The application and the
// bootloader answer at the same address (BL_I2C_BASE_ADDRESS), so one
// Transport covers both.
class Transport {
 public:
  virtual ~Transport() = default;
  // START, address+W, data, STOP.
  virtual bool write(const uint8_t *data, size_t len) = 0;
  // START, address+R, data, STOP. No register write first - the bootloader's
  // CRC result is polled this way.
  virtual bool read(uint8_t *data, size_t len) = 0;
  // Register-style read: write, repeated START, read.
  virtual bool write_read(const uint8_t *write_data, size_t write_len, uint8_t *read_data, size_t read_len) = 0;
};

enum class LogLevel : uint8_t { ERROR, WARN, INFO, CONFIG, DEBUG, VERBOSE };

class Platform {
 public:
  virtual ~Platform() = default;
  virtual uint32_t millis() = 0;
  // Only ever used for the short, bounded pauses between retries of one
  // transaction (single-digit to tens of milliseconds). Everything that waits
  // longer is a polled state, so a host's main loop keeps running.
  virtual void delay_ms(uint32_t ms) = 0;
  // Optional. The default drops everything.
  virtual void log(LogLevel level, const char *format, va_list args) {
    (void) level;
    (void) format;
    (void) args;
  }
};

#if defined(__GNUC__)
#define FADERBUDDY_PRINTF_FORMAT(fmt_index, args_index) __attribute__((format(printf, fmt_index, args_index)))
#else
#define FADERBUDDY_PRINTF_FORMAT(fmt_index, args_index)
#endif

void logf(Platform &platform, LogLevel level, const char *format, ...) FADERBUDDY_PRINTF_FORMAT(3, 4);

}  // namespace faderbuddy
