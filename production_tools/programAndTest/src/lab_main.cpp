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
 * Control-loop tuning harness ("lab" env) for the programAndTest jig.
 *
 * Replaces the production test state machine with a line-oriented serial
 * command interface so a host script can command motion and capture
 * high-rate position traces for step-response analysis.
 *
 * Build/flash:  pio run -e lab --target upload
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

#include "Adafruit_INA3221.h"
#include "fader_buddy_i2c.h"

#define PIN_LED_RED 21
#define PIN_LED_GREEN 22

#define PIN_INA_SCL 13
#define PIN_INA_SDA 17

#define PIN_FADER_BUDDY_SCL 27
#define PIN_FADER_BUDDY_SDA 26

#define PIN_SERVO 32
#define SERVO_TOUCH_POS 88
#define SERVO_CLEAR_POS 50

// INA3221 channels: 0 = 3.3V logic rail, 1 = 5V motor rail
#define INA_CH_LOGIC 0
#define INA_CH_MOTOR 1

Adafruit_INA3221 ina3221;
TwoWire WireFaderBuddy = TwoWire(1);
FaderBuddyI2C faderBuddy;
Servo servo;

bool inaPresent = false;

// ---------------------------------------------------------------------------
// Trace buffer
// ---------------------------------------------------------------------------
#define MAX_SAMPLES 6000

struct Sample {
  uint32_t t_us;
  uint32_t state;
};

Sample samples[MAX_SAMPLES];
uint16_t sampleCount = 0;
uint16_t stopSampleIndex = 0;  // index at which drive was cut (stopdist)

// Motor rail current is sampled on a slower cadence (separate I2C bus, slow ADC)
#define MAX_CURRENT_SAMPLES 400
struct CurrentSample {
  uint32_t t_us;
  float mA;
};
CurrentSample currentSamples[MAX_CURRENT_SAMPLES];
uint16_t currentSampleCount = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline uint8_t stateMode(uint32_t s) {
  return (s & STATE_MODE_bm) >> STATE_MODE_bp;
}
static inline uint8_t statePosition(uint32_t s) {
  return (s & STATE_POSITION_bm) >> STATE_POSITION_bp;
}
static inline uint16_t stateRawAdc(uint32_t s) {
  return (s & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp;
}

// Capture position samples for `durationMs`, sampling motor current every
// `currentPeriodMs` (0 disables current sampling).
void capture(uint32_t durationMs, uint32_t currentPeriodMs) {
  sampleCount = 0;
  currentSampleCount = 0;
  uint32_t startUs = micros();
  uint32_t endUs = startUs + durationMs * 1000UL;
  uint32_t nextCurrentUs = startUs;

  while ((int32_t)(micros() - endUs) < 0 && sampleCount < MAX_SAMPLES) {
    uint32_t s;
    uint32_t t = micros();
    if (faderBuddy.readState(s)) {
      samples[sampleCount].t_us = t - startUs;
      samples[sampleCount].state = s;
      sampleCount++;
    }
    if (currentPeriodMs > 0 && inaPresent &&
        (int32_t)(micros() - nextCurrentUs) >= 0 &&
        currentSampleCount < MAX_CURRENT_SAMPLES) {
      uint32_t tc = micros();
      currentSamples[currentSampleCount].t_us = tc - startUs;
      currentSamples[currentSampleCount].mA =
          ina3221.getCurrentAmps(INA_CH_MOTOR) * 1000.0f;
      currentSampleCount++;
      nextCurrentUs += currentPeriodMs * 1000UL;
    }
  }
}

void dumpTrace() {
  Serial.println("#BEGIN");
  Serial.print("#samples=");
  Serial.println(sampleCount);
  Serial.print("#stop_index=");
  Serial.println(stopSampleIndex);
  Serial.println("#cols=t_us,raw_adc,pos,mode,touch");
  for (uint16_t i = 0; i < sampleCount; i++) {
    uint32_t s = samples[i].state;
    Serial.print(samples[i].t_us);
    Serial.print(',');
    Serial.print(stateRawAdc(s));
    Serial.print(',');
    Serial.print(statePosition(s));
    Serial.print(',');
    Serial.print(stateMode(s));
    Serial.print(',');
    Serial.println((s & STATE_TOUCH_bm) ? 1 : 0);
  }
  if (currentSampleCount > 0) {
    Serial.println("#current_cols=t_us,mA");
    for (uint16_t i = 0; i < currentSampleCount; i++) {
      Serial.print("I,");
      Serial.print(currentSamples[i].t_us);
      Serial.print(',');
      Serial.println(currentSamples[i].mA, 2);
    }
  }
  Serial.println("#END");
}

// Raw open-loop drive command (DEBUG_DRIVE firmware builds only)
bool debugDrive(uint8_t flags, uint8_t duty) {
  WireFaderBuddy.beginTransmission(FADER_BUDDY_I2C_ADDR);
  WireFaderBuddy.write(REG_DEBUG_DRIVE);
  WireFaderBuddy.write(flags);
  WireFaderBuddy.write(duty);
  return WireFaderBuddy.endTransmission() == 0;
}

// Capture while holding an open-loop drive, refreshing it often enough to keep
// the firmware's command watchdog satisfied.
void captureOpenLoop(uint8_t flags, uint8_t duty, uint32_t durationMs,
                     uint32_t currentPeriodMs) {
  sampleCount = 0;
  currentSampleCount = 0;
  uint32_t startUs = micros();
  uint32_t endUs = startUs + durationMs * 1000UL;
  uint32_t nextCurrentUs = startUs;
  uint32_t nextRefreshUs = startUs;

  while ((int32_t)(micros() - endUs) < 0 && sampleCount < MAX_SAMPLES) {
    if ((int32_t)(micros() - nextRefreshUs) >= 0) {
      debugDrive(flags, duty);
      nextRefreshUs += 150000UL;  // watchdog is 400 ms
    }
    uint32_t s;
    uint32_t t = micros();
    if (faderBuddy.readState(s)) {
      samples[sampleCount].t_us = t - startUs;
      samples[sampleCount].state = s;
      sampleCount++;
    }
    if (currentPeriodMs > 0 && inaPresent &&
        (int32_t)(micros() - nextCurrentUs) >= 0 &&
        currentSampleCount < MAX_CURRENT_SAMPLES) {
      uint32_t tc = micros();
      currentSamples[currentSampleCount].t_us = tc - startUs;
      currentSamples[currentSampleCount].mA =
          ina3221.getCurrentAmps(INA_CH_MOTOR) * 1000.0f;
      currentSampleCount++;
      nextCurrentUs += currentPeriodMs * 1000UL;
    }
  }
  debugDrive(DEBUG_DRIVE_FLAGS_EXIT, 0);
}

// Drive open-loop for driveMs, then switch to stopFlags (coast or brake) for
// the remainder, logging throughout. Used to measure stopping distance.
void captureStopDistance(uint8_t flags, uint8_t duty, uint32_t driveMs,
                         uint8_t stopFlags, uint32_t totalMs) {
  sampleCount = 0;
  currentSampleCount = 0;
  uint32_t startUs = micros();
  uint32_t endUs = startUs + totalMs * 1000UL;
  uint32_t switchUs = startUs + driveMs * 1000UL;
  uint32_t nextRefreshUs = startUs;
  bool stopped = false;

  while ((int32_t)(micros() - endUs) < 0 && sampleCount < MAX_SAMPLES) {
    if (!stopped && (int32_t)(micros() - switchUs) >= 0) {
      debugDrive(stopFlags, 0);
      stopped = true;
      nextRefreshUs = micros();
      stopSampleIndex = sampleCount;
    }
    if ((int32_t)(micros() - nextRefreshUs) >= 0) {
      debugDrive(stopped ? stopFlags : flags, stopped ? 0 : duty);
      nextRefreshUs += 150000UL;
    }
    uint32_t s;
    uint32_t t = micros();
    if (faderBuddy.readState(s)) {
      samples[sampleCount].t_us = t - startUs;
      samples[sampleCount].state = s;
      sampleCount++;
    }
  }
  debugDrive(DEBUG_DRIVE_FLAGS_EXIT, 0);
}

// Wait until the fader reports INPUT_IDLE (or timeout), without logging.
bool waitIdle(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint32_t s;
    if (faderBuddy.readState(s) && stateMode(s) == MODE_INPUT_IDLE) {
      return true;
    }
    delay(5);
  }
  return false;
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------

char cmdBuf[128];
uint8_t cmdLen = 0;

void printState() {
  uint32_t s;
  if (!faderBuddy.readState(s)) {
    Serial.println("ERR read state");
    return;
  }
  Serial.print("OK state=0x");
  Serial.print(s, HEX);
  Serial.print(" mode=");
  Serial.print(stateMode(s));
  Serial.print(" pos=");
  Serial.print(statePosition(s));
  Serial.print(" raw=");
  Serial.print(stateRawAdc(s));
  Serial.print(" touch=");
  Serial.println((s & STATE_TOUCH_bm) ? 1 : 0);
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " ");
  if (cmd == nullptr) return;

  if (strcmp(cmd, "ping") == 0) {
    Serial.println("OK pong");

  } else if (strcmp(cmd, "ver") == 0) {
    uint8_t v;
    if (faderBuddy.readProtocolVersion(v)) {
      Serial.print("OK version=");
      Serial.println(v);
    } else {
      Serial.println("ERR read version");
    }

  } else if (strcmp(cmd, "state") == 0) {
    printState();

  } else if (strcmp(cmd, "target") == 0) {
    char* a = strtok(nullptr, " ");
    if (a == nullptr) { Serial.println("ERR usage: target <0-255>"); return; }
    uint8_t t = atoi(a);
    Serial.println(faderBuddy.writeTargetPosition(t) ? "OK" : "ERR write");

  } else if (strcmp(cmd, "sstep") == 0) {
    // sstep <from> <to> <speed> <log_ms>  - step with the optional speed limit
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    char* d = strtok(nullptr, " ");
    if (!a || !b || !c || !d) { Serial.println("ERR usage: sstep <from> <to> <speed> <log_ms>"); return; }
    uint8_t speed = (uint8_t)atoi(c);
    // move to the start position at full speed
    WireFaderBuddy.beginTransmission(FADER_BUDDY_I2C_ADDR);
    WireFaderBuddy.write(REG_LAYER_TARGET);
    WireFaderBuddy.write((uint8_t)0);
    WireFaderBuddy.write((uint8_t)atoi(a));
    WireFaderBuddy.write((uint8_t)LAYER_MOVE_TIME_UNLIMITED);
    if (WireFaderBuddy.endTransmission() != 0) { Serial.println("ERR write from"); return; }
    waitIdle(6000);
    delay(500);
    // then the measured move, speed limited
    WireFaderBuddy.beginTransmission(FADER_BUDDY_I2C_ADDR);
    WireFaderBuddy.write(REG_LAYER_TARGET);
    WireFaderBuddy.write((uint8_t)0);
    WireFaderBuddy.write((uint8_t)atoi(b));
    WireFaderBuddy.write(speed);
    if (WireFaderBuddy.endTransmission() != 0) { Serial.println("ERR write to"); return; }
    capture(atol(d), 5);
    dumpTrace();
    Serial.println("OK");

  } else if (strcmp(cmd, "step") == 0) {
    // step <from> <to> <log_ms> [settle_ms]
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    char* d = strtok(nullptr, " ");
    if (a == nullptr || b == nullptr || c == nullptr) {
      Serial.println("ERR usage: step <from> <to> <log_ms> [settle_ms]");
      return;
    }
    uint8_t from = atoi(a);
    uint8_t to = atoi(b);
    uint32_t logMs = atol(c);
    uint32_t settleMs = (d != nullptr) ? atol(d) : 700;

    if (!faderBuddy.writeTargetPosition(from)) { Serial.println("ERR write from"); return; }
    waitIdle(4000);
    delay(settleMs);
    if (!faderBuddy.writeTargetPosition(to)) { Serial.println("ERR write to"); return; }
    capture(logMs, 5);
    dumpTrace();
    Serial.println("OK");

  } else if (strcmp(cmd, "drive") == 0) {
    // drive <flags> <duty>   (flags 255 = exit open-loop mode)
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    if (a == nullptr || b == nullptr) { Serial.println("ERR usage: drive <flags> <duty>"); return; }
    Serial.println(debugDrive((uint8_t)strtoul(a, nullptr, 0),
                              (uint8_t)strtoul(b, nullptr, 0)) ? "OK" : "ERR write");

  } else if (strcmp(cmd, "openloop") == 0) {
    // openloop <start_pos> <flags> <duty> <log_ms>
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    char* d = strtok(nullptr, " ");
    if (a == nullptr || b == nullptr || c == nullptr || d == nullptr) {
      Serial.println("ERR usage: openloop <start_pos> <flags> <duty> <log_ms>");
      return;
    }
    debugDrive(DEBUG_DRIVE_FLAGS_EXIT, 0);
    delay(5);
    if (!faderBuddy.writeTargetPosition((uint8_t)atoi(a))) { Serial.println("ERR write start"); return; }
    waitIdle(4000);
    delay(400);
    captureOpenLoop((uint8_t)strtoul(b, nullptr, 0), (uint8_t)strtoul(c, nullptr, 0),
                    atol(d), 5);
    dumpTrace();
    Serial.println("OK");

  } else if (strcmp(cmd, "stopdist") == 0) {
    // stopdist <start_pos> <flags> <duty> <drive_ms> <stop_flags> <total_ms>
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    char* d = strtok(nullptr, " ");
    char* e = strtok(nullptr, " ");
    char* f = strtok(nullptr, " ");
    if (!a || !b || !c || !d || !e || !f) {
      Serial.println("ERR usage: stopdist <start> <flags> <duty> <drive_ms> <stop_flags> <total_ms>");
      return;
    }
    debugDrive(DEBUG_DRIVE_FLAGS_EXIT, 0);
    delay(5);
    if (!faderBuddy.writeTargetPosition((uint8_t)atoi(a))) { Serial.println("ERR write start"); return; }
    waitIdle(4000);
    delay(400);
    stopSampleIndex = 0;
    captureStopDistance((uint8_t)strtoul(b, nullptr, 0), (uint8_t)strtoul(c, nullptr, 0),
                        atol(d), (uint8_t)strtoul(e, nullptr, 0), atol(f));
    dumpTrace();
    Serial.println("OK");

  } else if (strcmp(cmd, "log") == 0) {
    char* a = strtok(nullptr, " ");
    uint32_t logMs = (a != nullptr) ? atol(a) : 500;
    capture(logMs, 5);
    dumpTrace();
    Serial.println("OK");

  } else if (strcmp(cmd, "cal") == 0) {
    if (!faderBuddy.selfCalibrate()) { Serial.println("ERR write"); return; }
    delay(4000);
    printState();

  } else if (strcmp(cmd, "clearerr") == 0) {
    Serial.println(faderBuddy.clearError() ? "OK" : "ERR write");

  } else if (strcmp(cmd, "touchcal") == 0) {
    Serial.println(faderBuddy.calibrateTouch() ? "OK" : "ERR write");

  } else if (strcmp(cmd, "layer") == 0) {
    char* a = strtok(nullptr, " ");
    if (a == nullptr) { Serial.println("ERR usage: layer <0-7>"); return; }
    WireFaderBuddy.beginTransmission(FADER_BUDDY_I2C_ADDR);
    WireFaderBuddy.write(REG_ACTIVE_LAYER);
    WireFaderBuddy.write((uint8_t)atoi(a));
    Serial.println(WireFaderBuddy.endTransmission() == 0 ? "OK" : "ERR write");

  } else if (strcmp(cmd, "haptic") == 0) {
    // haptic <layer> <config16>
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    if (a == nullptr || b == nullptr) { Serial.println("ERR usage: haptic <layer> <cfg>"); return; }
    uint16_t cfg = (uint16_t)strtoul(b, nullptr, 0);
    WireFaderBuddy.beginTransmission(FADER_BUDDY_I2C_ADDR);
    WireFaderBuddy.write(REG_LAYER_HAPTIC_CONFIG);
    WireFaderBuddy.write((uint8_t)atoi(a));
    WireFaderBuddy.write((uint8_t)(cfg >> 8));
    WireFaderBuddy.write((uint8_t)(cfg & 0xFF));
    Serial.println(WireFaderBuddy.endTransmission() == 0 ? "OK" : "ERR write");

  } else if (strcmp(cmd, "servo") == 0) {
    char* a = strtok(nullptr, " ");
    if (a == nullptr) { Serial.println("ERR usage: servo <touch|clear|angle>"); return; }
    if (strcmp(a, "touch") == 0) servo.write(SERVO_TOUCH_POS);
    else if (strcmp(a, "clear") == 0) servo.write(SERVO_CLEAR_POS);
    else servo.write(atoi(a));
    Serial.println("OK");

  } else if (strcmp(cmd, "power") == 0) {
    if (!inaPresent) { Serial.println("ERR no ina3221"); return; }
    Serial.print("OK logic_v=");
    Serial.print(ina3221.getBusVoltage(INA_CH_LOGIC), 3);
    Serial.print(" logic_ma=");
    Serial.print(ina3221.getCurrentAmps(INA_CH_LOGIC) * 1000.0f, 2);
    Serial.print(" motor_v=");
    Serial.print(ina3221.getBusVoltage(INA_CH_MOTOR), 3);
    Serial.print(" motor_ma=");
    Serial.println(ina3221.getCurrentAmps(INA_CH_MOTOR) * 1000.0f, 2);

  } else if (strcmp(cmd, "touch") == 0) {
    uint16_t raw = 0, ref = 0, recal = 0;
    int16_t delta = 0;
    faderBuddy.readTouchRaw(raw);
    faderBuddy.readTouchDelta(delta);
    faderBuddy.readTouchReference(ref);
    faderBuddy.readTouchRecalCount(recal);
    uint32_t st = 0;
    faderBuddy.readState(st);
    Serial.print("OK touch_bit="); Serial.print((st & STATE_TOUCH_bm) ? 1 : 0);
    Serial.print(" raw="); Serial.print(raw);
    Serial.print(" delta="); Serial.print(delta);
    Serial.print(" ref="); Serial.print(ref);
    Serial.print(" recal="); Serial.println(recal);

  } else if (strcmp(cmd, "gain") == 0) {
    // gain <index> <value>   (KP/KD/deadband are x1000)
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    if (a == nullptr || b == nullptr) { Serial.println("ERR usage: gain <index> <value>"); return; }
    int16_t v = (int16_t)atoi(b);
    WireFaderBuddy.beginTransmission(FADER_BUDDY_I2C_ADDR);
    WireFaderBuddy.write(REG_DEBUG_GAINS);
    WireFaderBuddy.write((uint8_t)atoi(a));
    WireFaderBuddy.write((uint8_t)((uint16_t)v >> 8));
    WireFaderBuddy.write((uint8_t)((uint16_t)v & 0xFF));
    Serial.println(WireFaderBuddy.endTransmission() == 0 ? "OK" : "ERR write");

  } else if (strcmp(cmd, "dbg") == 0) {
    uint8_t d[18];
    WireFaderBuddy.beginTransmission(FADER_BUDDY_I2C_ADDR);
    WireFaderBuddy.write(REG_DEBUG_STATUS);
    if (WireFaderBuddy.endTransmission(false) != 0) { Serial.println("ERR write"); return; }
    if (WireFaderBuddy.requestFrom((uint8_t)FADER_BUDDY_I2C_ADDR, (size_t)18) != 18) {
      Serial.println("ERR read"); return;
    }
    for (uint8_t i = 0; i < 18; i++) d[i] = WireFaderBuddy.read();
    int16_t v[9];
    for (uint8_t i = 0; i < 9; i++) v[i] = (int16_t)(((uint16_t)d[i*2] << 8) | d[i*2+1]);
    Serial.print("OK calib_min="); Serial.print((uint16_t)v[0]);
    Serial.print(" calib_max="); Serial.print((uint16_t)v[1]);
    Serial.print(" target_adc="); Serial.print(v[2]);
    Serial.print(" drive="); Serial.print(v[3]);
    Serial.print(" vel="); Serial.print(v[4]);
    Serial.print(" err="); Serial.print(v[5] / 8.0f, 2);
    Serial.print(" loop_hz="); Serial.print((uint16_t)v[6]);
    Serial.print(" tick_hz="); Serial.print((uint16_t)v[7]);
    Serial.print(" vmax="); Serial.println((uint16_t)v[8]);

  } else if (strcmp(cmd, "rate") == 0) {
    // Measure achievable I2C poll rate
    uint32_t start = millis();
    uint16_t n = 0;
    uint32_t s;
    while (millis() - start < 200) {
      if (faderBuddy.readState(s)) n++;
    }
    Serial.print("OK poll_hz=");
    Serial.println(n * 5);

  } else {
    Serial.print("ERR unknown command: ");
    Serial.println(cmd);
  }
}

void setup() {
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, HIGH);

  Serial.begin(921600);

  Wire.begin(PIN_INA_SDA, PIN_INA_SCL);
  WireFaderBuddy.begin(PIN_FADER_BUDDY_SDA, PIN_FADER_BUDDY_SCL, 400000);
  faderBuddy.begin(&WireFaderBuddy);

  servo.attach(PIN_SERVO);
  servo.write(SERVO_CLEAR_POS);

  inaPresent = ina3221.begin(0x40, &Wire);
  if (inaPresent) {
    ina3221.setAveragingMode(INA3221_AVG_1_SAMPLE);
    for (uint8_t i = 0; i < 3; i++) {
      ina3221.setShuntResistance(i, 0.1);
    }
  }

  delay(200);
  Serial.println("#LAB READY");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}
