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

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include <LittleFS.h>

#include "Adafruit_INA3221.h"
#include "FaderBuddyHost.h"
#include "fader_app_image.h"

#define FlashFS LittleFS

#define PIN_LED_RED 21
#define PIN_LED_GREEN 22

#define PIN_PRESENCE_SWITCH 36

// LilyGO T-Display left button (active low). Press-and-hold toggles dummy mode.
#define PIN_BUTTON_LEFT 0
#define BUTTON_HOLD_MS 1000

#define PIN_PHOTODIODE_PWR 37
#define PIN_PHOTODIODE_DBG 38


#define PIN_INA_SCL 13
#define PIN_INA_SDA 17

#define PIN_FADER_BUDDY_SCL 27
#define PIN_FADER_BUDDY_SDA 26

#define PIN_SERVO 32
#define SERVO_TOUCH_POS 88
#define SERVO_CLEAR_POS 50

// Fixed "old firmware" baseline installed over I2C at the end of
// TEST_FW_BOOTSTRAP (see factory_test_images/old_app_fw0.hex and its README;
// embedded as FADER_OLD_APP_IMAGE). Must match the FW_VERSION baked into that
// checked-in image.
#define OLD_FW_VERSION_FOR_TEST (0)


// Bounded linear interpolation macro (float only)
#define LERP(x, in_min, in_max, out_min, out_max) \
  ({ \
    float _x = (float)(x); \
    float _in_min = (float)(in_min); \
    float _in_max = (float)(in_max); \
    float _out_min = (float)(out_min); \
    float _out_max = (float)(out_max); \
    _x = (_x < _in_min) ? _in_min : ((_x > _in_max) ? _in_max : _x); \
    (_out_min + (_out_max - _out_min) * (_x - _in_min) / (_in_max - _in_min)); \
  })

Adafruit_INA3221 ina3221;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

TwoWire WireFaderBuddy = TwoWire(1);  // Use second I2C peripheral
// The DUT, through the shared host library (hostLib/). The app and its
// bootloader answer at the same address. No library logging: this serial port
// carries the >>...<< protocol test_host.py parses, and the install runs on
// another core, so its lines could land in the middle of one.
faderbuddy::WireTransport faderTransport(WireFaderBuddy, BL_I2C_BASE_ADDRESS);
faderbuddy::ArduinoPlatform faderPlatform;
faderbuddy::FaderBuddyDevice faderBuddy(faderTransport, faderPlatform);
faderbuddy::FirmwareUpdater faderUpdater(faderBuddy);

uint32_t bootTime = 0;

Servo servo;

// ============================================================================
// Test State Machine
// ============================================================================
enum TestState {
  TEST_IDLE,              // Waiting for presence switch press
  TEST_LOGIC_POWER,       // Testing 3.3V logic rail
  TEST_MOTOR_POWER,       // Testing 5V motor rail
  TEST_POWER_LED,         // Testing power LED
  TEST_FW_BOOTSTRAP,      // UPDI-flash the bootloader, then install the fixed old app over I2C
  TEST_FW_I2C_UPDATE,     // I2C bootloader update to the current app + validate
  TEST_DEBUG_LED,         // Testing debug LED blink pattern
  TEST_SELF_CALIBRATION,  // Self-calibration test
  TEST_DIAGNOSTICS,       // I2C diagnostics (10 seconds)
  TEST_TOUCH_SENSOR,      // Touch sensor test with servo
  TEST_PASSED,            // All tests passed
  TEST_FAILED             // One or more tests failed
};

TestState currentTestState = TEST_IDLE;

bool bootloadActive();  // these are defined with the I2C install code below
void startBootloadStreamTask();
TestState lastReportedTestState = TEST_IDLE;  // Last state reported to host script over serial
bool lastPresenceState = false;

// Dummy mode: skips the firmware bootstrap/I2C-update steps (which need real
// UPDI hardware and a real fader) so the rest of the jig flow can be
// exercised without them. Toggled by holding the left button on the T-Display.
bool dummyMode = false;
bool lastButtonPressed = false;
uint32_t buttonPressStartTime = 0;
bool buttonHoldHandled = false;

// Motor fader version info (read once per test run)
struct FaderBuddyVersion {
  uint8_t protocolVersion;
  uint16_t fwVersion;
  bool valid;
} versionInfo = {0, 0, false};

// Motor fader state (read continuously during diagnostics)
struct FaderBuddyState {
  bool touch;
  uint8_t mode;
  uint8_t position;
  uint16_t rawADC;
  uint32_t uptime;
  int16_t touchDelta;
  uint16_t touchReference;
  uint16_t touchRecalCount;
  bool valid;
} faderState = {false, 0, 0, 0, 0, 0, 0, 0, false};

// Test tracking
struct TestTracking {
  uint32_t testStartTime;
  uint8_t diagnosticsMovementStep;

  // Datapoint reporting: when the current phase was entered, so PHASE_END can
  // carry its elapsed time.
  uint32_t phaseStartTime;

  // Peak motor-rail draw seen while the fader is actually being driven. Reset
  // when self-calibration starts, reported when the movement checks finish, so
  // it covers the whole of the heavy driving.
  float peakMotorCurrentMa;

  // Movement checks: how long each commanded move took to settle, and how far
  // off the commanded position it ended up. Both were previously measured only
  // against a timeout and then discarded.
  uint16_t settleMs[3];
  uint8_t settleError[3];

  // Touch checks: latency from servo actuation to detect, and from servo
  // release to clear, plus the delta seen while touched.
  uint16_t touchDetectMs;
  uint16_t touchReleaseMs;
  int16_t touchDeltaAtTouch;

  // Power test accumulators
  float v0Sum;
  float i0Sum;
  float v1Sum;
  float i1Sum;
  uint16_t sampleCount;

  // Firmware upload tracking
  enum FirmwareUploadPhase {
    FIRMWARE_PHASE_PING,
    FIRMWARE_PHASE_UPLOAD,
    FIRMWARE_PHASE_WAIT_BOOTLOADER,  // confirm the UPDI-flashed bootloader is resident with no app
    FIRMWARE_PHASE_INSTALL_OLD,      // I2C-install the fixed old app, confirm OLD_FW_VERSION_FOR_TEST
    FIRMWARE_PHASE_BOOTLOAD_UPDATE,  // drive the I2C bootloader to the current app image
    FIRMWARE_PHASE_READ_SERIAL,
    FIRMWARE_PHASE_COMPLETE
  };
  FirmwareUploadPhase firmwarePhase;
  uint32_t firmwarePhaseStartTime;
  bool firmwareCommandSent;
  char serialBuffer[256];  // Buffer for incoming serial data
  uint16_t serialBufferPos;  // Current write position in buffer

  // Debug LED test tracking
  uint32_t debugLedTestStartTime;
  uint8_t debugLedTransitionCount;
  bool debugLedState;  // Current state for hysteresis tracking

  // Self-calibration test tracking
  uint32_t selfCalStartTime;
  bool selfCalRequestSent;
  bool selfCalEnteredMode;
  uint16_t selfCalMinADC = 0xFFFF;  // Sentinel for "no sample yet"
  uint16_t selfCalMaxADC;

  // Touch sensor test tracking
  enum TouchTestPhase {
    TOUCH_PHASE_CHECK_NO_TOUCH,
    TOUCH_PHASE_MOVE_TO_POSITION,
    TOUCH_PHASE_WAIT_FOR_IDLE,
    TOUCH_PHASE_SERVO_TOUCH,
    TOUCH_PHASE_WAIT_FOR_TOUCH_DETECT,
    TOUCH_PHASE_CONFIRM_TOUCH,
    TOUCH_PHASE_SERVO_CLEAR,
    TOUCH_PHASE_WAIT_FOR_TOUCH_CLEAR,
    TOUCH_PHASE_FINAL_WAIT,
    TOUCH_PHASE_CLEANUP_ON_FAILURE
  };
  TouchTestPhase touchTestPhase;
  uint32_t touchTestPhaseStartTime;

  // Test results
  String failedTestName;

  // I2C bootloader install (stepBootload(), used by both
  // FIRMWARE_PHASE_INSTALL_OLD and FIRMWARE_PHASE_BOOTLOAD_UPDATE). The install
  // itself runs in its own task; these track it for loop() and the display.
  bool bootloadStarted;           // stepBootload() has started the current image
  uint32_t bootloadStartTime;     // for the whole-install watchdog
  uint32_t bootloadPageIndex;
  uint32_t bootloadTotalPages;
  // No aggregate initializer: this has static storage duration, so every member
  // is zero-initialized (which is the right start for all of them - the phase
  // enums both begin at 0), and the one member wanting a non-zero default
  // declares it inline above. A positional list here silently went wrong every
  // time a field was inserted.
} testTracking;

void setup() {
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_PRESENCE_SWITCH, INPUT);
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);

  pinMode(PIN_PHOTODIODE_PWR, INPUT);
  pinMode(PIN_PHOTODIODE_DBG, INPUT);

  // Configure LED PWM for high resolution (10-bit = 1024 levels)
  analogWriteResolution(10);

  Serial.begin(115200);
  Wire.begin(PIN_INA_SDA, PIN_INA_SCL);

  // Initialize motor fader I2C on separate bus
  WireFaderBuddy.begin(PIN_FADER_BUDDY_SDA, PIN_FADER_BUDDY_SCL);
  WireFaderBuddy.setClock(100000);
  WireFaderBuddy.setTimeOut(250);  // tolerate bootloader CRC-compute clock stretching
  startBootloadStreamTask();

  // Initialize servo - move to clear position and disable after 2 seconds
  servo.attach(PIN_SERVO);
  servo.write(SERVO_CLEAR_POS);
  // delay(2000);  // Allow servo to reach clear position
  // servo.detach();

  // Initialize display
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  sprite.createSprite(135, 240);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.setTextSize(2);


  delay(100);

  // Initialize the INA3221
  if (!ina3221.begin(0x40, &Wire)) { // can use other I2C addresses or buses
    Serial.println("Failed to find INA3221 chip");
    while (1) {
      digitalWrite(PIN_LED_RED, HIGH);
      delay(200);
      digitalWrite(PIN_LED_RED, LOW);
      delay(200);
    }
  }
  Serial.println("INA3221 Found!");

  ina3221.setAveragingMode(INA3221_AVG_4_SAMPLES);

  // Set shunt resistances for all channels to 0.1 ohms
  for (uint8_t i = 0; i < 3; i++) {
    ina3221.setShuntResistance(i, 0.1);
  }

  // Initialize LittleFS for smooth font loading
  if (!LittleFS.begin()) {
    Serial.println("Flash FS initialisation failed!");
    while (1) {
      digitalWrite(PIN_LED_RED, HIGH);
      delay(100);
      digitalWrite(PIN_LED_RED, LOW);
      delay(100);
    }
  }
  Serial.println("Flash FS available!");

  // Check if roboto font exists
  if (LittleFS.exists("/roboto_14.vlw") == false) {
    Serial.println("Roboto font missing in Flash FS, did you upload it?");
    while (1) {
      digitalWrite(PIN_LED_RED, HIGH);
      delay(500);
      digitalWrite(PIN_LED_RED, LOW);
      delay(500);
    }
  } else {
    Serial.println("Roboto font found OK.");
  }

  // Load the Roboto smooth font once during setup
  sprite.loadFont("roboto_14", LittleFS); 

  // Record boot time for delayed programming trigger
  bootTime = millis();
}


const char* getModeString(uint8_t mode) {
  switch (mode) {
    case MODE_REMOTE_MOVEMENT_IN_PROGRESS: return "REMOTE";
    case MODE_INPUT_ACTIVE: return "INPUT_ACT";
    case MODE_INPUT_IDLE: return "INPUT_IDL";
    case MODE_ERROR: return "ERROR";
    case MODE_SELF_CALIBRATION: return "SELF_CAL";
    default: return "UNKNOWN";
  }
}

// ============================================================================
// Machine-readable datapoint reporting (parsed by test_host.py)
// ============================================================================
//
//   >>PHASE_START:<phase><<
//   >>DATA:<phase>:<key>=<value>:<unit>:<min>:<max>:<axis_lo>:<axis_hi><<
//   >>PHASE_END:<phase>:<PASS|FAIL>:<elapsed_ms><<
//
// Limits travel with the value, so the host never has to duplicate the pass
// criteria enforced here; an empty min or max means that side is unbounded.
// Adding a datapoint is then a firmware-only change - the report renders
// whatever arrives without knowing what any of it means.
//
// axis_lo/axis_hi are the full meaningful range of the measurement, which is
// what the report plots the value against so the passing window is visible in
// proportion to everything the sensor could have read. Only the firmware knows
// these (an ADC's full scale, a timeout's ceiling), so they are reported rather
// than guessed host-side. Both may be empty, in which case the host falls back
// to the limits.
//
// These sit alongside the existing human-readable prints rather than replacing
// them: the serial log is still what you read when debugging at the bench.

void reportPhaseStart(const char* phase) {
  Serial.printf(">>PHASE_START:%s<<\n", phase);
}

void reportPhaseEnd(const char* phase, bool passed, uint32_t elapsedMs) {
  Serial.printf(">>PHASE_END:%s:%s:%lu<<\n", phase, passed ? "PASS" : "FAIL",
                (unsigned long)elapsedMs);
}

void reportDataF(const char* phase, const char* key, float value, uint8_t decimals,
                 const char* unit, const char* lo, const char* hi,
                 const char* axisLo = "", const char* axisHi = "") {
  Serial.printf(">>DATA:%s:%s=%.*f:%s:%s:%s:%s:%s<<\n", phase, key, decimals, value, unit,
                lo, hi, axisLo, axisHi);
}

void reportDataI(const char* phase, const char* key, long value,
                 const char* unit, const char* lo, const char* hi,
                 const char* axisLo = "", const char* axisHi = "") {
  Serial.printf(">>DATA:%s:%s=%ld:%s:%s:%s:%s:%s<<\n", phase, key, value, unit,
                lo, hi, axisLo, axisHi);
}

void reportDataStr(const char* phase, const char* key, const char* value) {
  Serial.printf(">>DATA:%s:%s=%s:::::<<\n", phase, key, value);
}

// Stable machine name for a test state, or nullptr for states that aren't a
// measured phase (idle, passed, failed).
const char* getTestPhaseKey(TestState state) {
  switch (state) {
    case TEST_LOGIC_POWER: return "LOGIC_POWER";
    case TEST_MOTOR_POWER: return "MOTOR_POWER";
    case TEST_POWER_LED: return "POWER_LED";
    case TEST_FW_BOOTSTRAP: return "FW_BOOTSTRAP";
    case TEST_FW_I2C_UPDATE: return "FW_I2C_UPDATE";
    case TEST_DEBUG_LED: return "DEBUG_LED";
    case TEST_SELF_CALIBRATION: return "SELF_CALIBRATION";
    case TEST_DIAGNOSTICS: return "MOVEMENT";
    case TEST_TOUCH_SENSOR: return "TOUCH_SENSOR";
    default: return nullptr;
  }
}

// Read REG_MOTOR_CAL and report what self-calibration measured about this
// unit's motor. Diagnostic only - nothing here is a pass/fail criterion, but
// it is the most genuinely per-unit data the board can report, and it is what
// makes a stiff or sloppy fader recognisable months later.
void reportMotorCal() {
  const char* PH = "MOTOR_CAL";
  uint32_t started = millis();
  uint8_t cal[12];

  reportPhaseStart(PH);
  if (!faderBuddy.read_motor_cal(cal)) {
    Serial.println("Failed to read motor characterisation (REG_MOTOR_CAL)");
    reportPhaseEnd(PH, false, millis() - started);
    return;
  }

  // Layout per i2c_data.h: valid, breakaway r/f, k r/f, v_jump r/f (u16 BE),
  // vel_min (u16 BE), deadband.
  uint16_t vjumpRising = ((uint16_t)cal[5] << 8) | cal[6];
  uint16_t vjumpFalling = ((uint16_t)cal[7] << 8) | cal[8];
  uint16_t velMin = ((uint16_t)cal[9] << 8) | cal[10];

  reportDataI(PH, "cal_valid", cal[0], "", "1", "1");
  reportDataI(PH, "breakaway_rising", cal[1], "/255", "", "", "0", "255");
  reportDataI(PH, "breakaway_falling", cal[2], "/255", "", "", "0", "255");
  reportDataI(PH, "k_rising", cal[3], "ADC/s", "", "");
  reportDataI(PH, "k_falling", cal[4], "ADC/s", "", "");
  reportDataI(PH, "vjump_rising", vjumpRising, "ADC/s", "", "");
  reportDataI(PH, "vjump_falling", vjumpFalling, "ADC/s", "", "");
  reportDataI(PH, "vel_min", velMin, "ADC/s", "", "");
  reportDataI(PH, "deadband", cal[11], "ADC", "", "");

  Serial.printf("Motor cal: valid=%u breakaway=%u/%u k=%u/%u vjump=%u/%u "
                "vel_min=%u deadband=%u\n",
                cal[0], cal[1], cal[2], cal[3], cal[4],
                vjumpRising, vjumpFalling, velMin, cal[11]);

  // cal_valid == 0 means the unit is running compiled-in defaults rather than
  // its own measurement, which is worth surfacing even though no check fails.
  reportPhaseEnd(PH, cal[0] == 1, millis() - started);
}

const String getTestStateName(TestState state) {
  switch (state) {
    case TEST_IDLE: return "IDLE";
    case TEST_LOGIC_POWER: return "1: LOGIC PWR";
    case TEST_MOTOR_POWER: return "2: MOTOR PWR";
    case TEST_POWER_LED: return "3: PWR LED";
    case TEST_FW_BOOTSTRAP: return "4: FW BOOTSTRAP";
    case TEST_FW_I2C_UPDATE: return "5: FW I2C UP";
    case TEST_DEBUG_LED: return "6: DBG LED";
    case TEST_SELF_CALIBRATION: return "7: SELF CAL";
    case TEST_DIAGNOSTICS: return "8: I2C DIAG";
    case TEST_TOUCH_SENSOR: return "9: TOUCH";
    case TEST_PASSED: return "PASSED";
    case TEST_FAILED:
      if (!testTracking.failedTestName.isEmpty()) {
        return String("FAIL: ") + testTracking.failedTestName;
      }
      return "FAILED";
    default: return "UNKNOWN";
  }
}

uint16_t getTestStateColor(TestState state) {
  switch (state) {
    case TEST_IDLE: return TFT_DARKGREY;
    case TEST_PASSED: return TFT_GREEN;
    case TEST_FAILED: return TFT_RED;
    default: return TFT_BLUE;  // Testing states
  }
}

// Tiny 3x5 pixel bitmap font, only the glyphs needed to spell "DUMMY" -- used
// for the dummy-mode indicator, which needs to be much smaller than the
// smallest size the loaded smooth font (roboto_14) can render.
const uint8_t TINY_FONT_D[5] = {0b110, 0b101, 0b101, 0b101, 0b110};
const uint8_t TINY_FONT_U[5] = {0b101, 0b101, 0b101, 0b101, 0b010};
const uint8_t TINY_FONT_M[5] = {0b101, 0b111, 0b101, 0b101, 0b101};
const uint8_t TINY_FONT_Y[5] = {0b101, 0b101, 0b010, 0b010, 0b010};

const uint8_t* tinyFontGlyph(char c) {
  switch (c) {
    case 'D': return TINY_FONT_D;
    case 'U': return TINY_FONT_U;
    case 'M': return TINY_FONT_M;
    case 'Y': return TINY_FONT_Y;
    default: return nullptr;
  }
}

// Draws text using the tiny 3x5 font directly onto the sprite (bypasses the
// loaded smooth font entirely, so it doesn't need swapping in/out per frame).
void drawTinyText(const char* text, int x, int y, uint16_t color) {
  int cx = x;
  for (const char* p = text; *p; p++) {
    const uint8_t* glyph = tinyFontGlyph(*p);
    if (glyph) {
      for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 3; col++) {
          if (glyph[row] & (0b100 >> col)) {
            sprite.drawPixel(cx + col, y + row, color);
          }
        }
      }
    }
    cx += 4;  // 3px glyph + 1px space
  }
}

uint8_t render_count = 0;
void updateDisplay(float v0, float c0, float v1, float c1) {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);

  // Status bar at top (40px tall)
  uint16_t statusColor = getTestStateColor(currentTestState);
  sprite.fillRect(0, 0, 135, 40, statusColor);

  // Center text in status bar
  uint16_t fg_color = currentTestState == TEST_PASSED ? TFT_BLACK : TFT_WHITE;
  sprite.setTextColor(fg_color, statusColor);
  String stateName = getTestStateName(currentTestState);
  int textWidth = sprite.textWidth(stateName);
  int textX = (currentTestState == TEST_IDLE || currentTestState == TEST_PASSED || currentTestState == TEST_FAILED) ? (135 - textWidth) / 2 : 10;
  sprite.setCursor(textX, 12);  // Vertically centered in 40px bar
  sprite.print(stateName);

  // I2C bootloader install progress bar, along the bottom edge of the status
  // bar (only meaningful while an install is running - see bootloadActive()).
  // Erase/verify/run/etc. show a full bar; page streaming fills it
  // incrementally as it runs.
  if (bootloadActive()) {
    uint32_t done = testTracking.bootloadPageIndex;
    if ((uint8_t)faderUpdater.stage() > (uint8_t)faderbuddy::UpdateStage::WRITE) {
      done = testTracking.bootloadTotalPages;
    }
    int barX = 4, barY = 34, barW = 127, barH = 5;
    sprite.drawRect(barX, barY, barW, barH, TFT_WHITE);
    int fillW = (int)((uint32_t)(barW - 2) * done / testTracking.bootloadTotalPages);
    if (fillW > 0) {
      sprite.fillRect(barX + 1, barY + 1, fillW, barH - 2, TFT_WHITE);
    }
  }

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);

  // Top-right readout, just below the status bar. During an I2C install the
  // protocol/firmware version aren't queryable (the device is mid-update), so
  // that slot shows page-write progress instead; otherwise it shows the
  // protocol/firmware version on two lines, hidden entirely until known.
  if (bootloadActive()) {
    char progBuf[16];
    snprintf(progBuf, sizeof(progBuf), "%lu/%lu", (unsigned long)testTracking.bootloadPageIndex,
             (unsigned long)testTracking.bootloadTotalPages);
    int w = sprite.textWidth(progBuf);
    sprite.setCursor(135 - w - 2, 44);
    sprite.print(progBuf);
  } else if (versionInfo.valid) {
    char protoBuf[16], fwBuf[16];
    snprintf(protoBuf, sizeof(protoBuf), "P:v%u", versionInfo.protocolVersion);
    snprintf(fwBuf, sizeof(fwBuf), "FW:v%u", versionInfo.fwVersion);
    int protoWidth = sprite.textWidth(protoBuf);
    int fwWidth = sprite.textWidth(fwBuf);
    sprite.setCursor(135 - protoWidth - 2, 44);
    sprite.print(protoBuf);
    sprite.setCursor(135 - fwWidth - 2, 58);
    sprite.print(fwBuf);
  }

  // TODO: move IO out of this method
  uint16_t pwr_led = analogRead(PIN_PHOTODIODE_PWR);
  if (pwr_led < 2800) {
    sprite.fillSmoothCircle(10, 58, 3, TFT_RED, TFT_BLACK);
  }
  sprite.setCursor(20, 50);
  sprite.print(pwr_led);

  uint16_t dbg_led = analogRead(PIN_PHOTODIODE_DBG);
  if (dbg_led < 2800) {
    sprite.fillSmoothCircle(10, 78, 3, TFT_RED, TFT_BLACK);
  }
  sprite.setCursor(20, 70);
  sprite.print(dbg_led);

  // Prepare voltage and current strings
  String v0_str = String(v0, 1) + "V";
  String c0_str = String((int)c0) + "mA";
  String v1_str = String(v1, 1) + "V";
  String c1_str = String((int)c1) + "mA";

  // Calculate right-aligned positions for channel 0 (left half, right align at x=67)
  int v0_width = sprite.textWidth(v0_str);
  int c0_width = sprite.textWidth(c0_str);
  sprite.setCursor(67 - v0_width, 200);
  sprite.print(v0_str);
  sprite.setCursor(67 - c0_width, 215);
  sprite.print(c0_str);
  int c0h = LERP(c0, 0, 15, 0, 240);
  sprite.fillRect(0, 240 - c0h, 2, c0h, TFT_GREEN);

  // Calculate right-aligned positions for channel 1 (right half, right align at x=135)
  int v1_width = sprite.textWidth(v1_str);
  int c1_width = sprite.textWidth(c1_str);
  sprite.setCursor(130 - v1_width, 200);
  sprite.print(v1_str);
  sprite.setCursor(130 - c1_width, 215);
  sprite.print(c1_str);
  int c1h = LERP(c1, 0, 300, 0, 240);
  sprite.fillRect(133, 240 - c1h, 2, c1h, TFT_RED);


  // Display motor fader diagnostics if available
  if (faderState.valid) {
    sprite.setCursor(5, 110);
    sprite.setTextColor(TFT_CYAN, TFT_BLACK);
    sprite.print("Up:");
    sprite.print(faderState.uptime / 1000.0, 1);
    sprite.print("s");

    sprite.setCursor(5, 130);
    sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
    sprite.print(getModeString(faderState.mode));

    sprite.setCursor(5, 150);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.print("Pos:");
    sprite.print(faderState.position);
    sprite.print(" ADC:");
    sprite.print(faderState.rawADC);

    sprite.setCursor(5, 170);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.print("Tch:");
    sprite.setTextColor(faderState.touch ? TFT_GREEN : TFT_RED, TFT_BLACK);
    sprite.print(faderState.touch ? "Y" : "N");

    // Touch diagnostics (delta, reference, recal count)
    sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    sprite.print(" d:");
    sprite.print(faderState.touchDelta);
    sprite.print(" r:");
    sprite.print(faderState.touchReference);
    sprite.print(" rc:");
    sprite.print(faderState.touchRecalCount);
  }

  render_count++;
  if (render_count & 0x01) {
    sprite.fillSmoothCircle(125, 235, 2, TFT_WHITE, TFT_BLACK);
  }

  // Dummy mode indicator: tiny label along the very bottom edge of the screen.
  if (dummyMode) {
    drawTinyText("DUMMY", 2, 240 - 5, TFT_YELLOW);
  }

  sprite.pushSprite(0, 0);
}


// ============================================================================
// Test Functions
// ============================================================================

bool testLogicPower(float v0, float i0) {
  // Accumulate samples over 1 second
  if (testTracking.sampleCount == 0) {
    testTracking.testStartTime = millis();
    testTracking.v0Sum = 0;
    testTracking.i0Sum = 0;
  }

  testTracking.v0Sum += v0;
  testTracking.i0Sum += i0;
  testTracking.sampleCount++;

  // Check if 1 second has elapsed
  if (millis() - testTracking.testStartTime < 1000) {
    return false;  // Still collecting samples
  }

  // Calculate averages
  float avgV0 = testTracking.v0Sum / testTracking.sampleCount;
  float avgI0 = testTracking.i0Sum / testTracking.sampleCount;

  Serial.print("Logic Power - Avg V0: ");
  Serial.print(avgV0, 3);
  Serial.print("V, Avg I0: ");
  Serial.print(avgI0, 2);
  Serial.println("mA");

  reportDataF("LOGIC_POWER", "logic_voltage", avgV0, 3, "V", "3.1", "3.5", "0", "3.6");
  reportDataF("LOGIC_POWER", "logic_current", avgI0, 2, "mA", "", "15.0", "0", "25");

  // Check ranges: 3.1-3.5V, 5-15mA
  if (avgV0 < 3.1 || avgV0 > 3.5) {
    testTracking.failedTestName = "LOG VOLT";
    Serial.println("FAILED: Logic voltage out of range");
  } else if (avgI0 > 15.0) {
    testTracking.failedTestName = "LOG CUR";
    Serial.println("FAILED: Logic current out of range");
  } else {
    Serial.println("PASSED: Logic power OK");
  }

  return true;  // Test complete
}

bool testMotorPower(float v1, float i1) {
  // Accumulate samples over 1 second
  if (testTracking.sampleCount == 0) {
    testTracking.testStartTime = millis();
    testTracking.v1Sum = 0;
    testTracking.i1Sum = 0;
  }

  testTracking.v1Sum += v1;
  testTracking.i1Sum += i1;
  testTracking.sampleCount++;

  // Check if 1 second has elapsed
  if (millis() - testTracking.testStartTime < 1000) {
    return false;  // Still collecting samples
  }

  // Calculate averages
  float avgV1 = testTracking.v1Sum / testTracking.sampleCount;
  float avgI1 = testTracking.i1Sum / testTracking.sampleCount;

  Serial.print("Motor Power - Avg V1: ");
  Serial.print(avgV1, 3);
  Serial.print("V, Avg I1: ");
  Serial.print(avgI1, 2);
  Serial.println("mA");

  reportDataF("MOTOR_POWER", "motor_voltage", avgV1, 3, "V", "4.3", "5.5", "0", "6.0");
  reportDataF("MOTOR_POWER", "motor_idle_current", avgI1, 2, "mA", "", "10.0", "0", "25");

  // Check ranges: 4.3-5.5V, 0-10mA
  if (avgV1 < 4.3 || avgV1 > 5.5) {
    testTracking.failedTestName = "MOT VOLT";
    Serial.println("FAILED: Motor voltage out of range");
  } else if (avgI1 < 0.0 || avgI1 > 10.0) {
    testTracking.failedTestName = "MOT CUR";
    Serial.println("FAILED: Motor current out of range");
  } else {
    Serial.println("PASSED: Motor power OK");
  }

  return true;  // Test complete
}

bool testPowerLED() {
  uint16_t pwr_led = analogRead(PIN_PHOTODIODE_PWR);

  Serial.print("Power LED - Photodiode ADC: ");
  Serial.println(pwr_led);

  // Lower reading = brighter LED, so this is an upper bound.
  reportDataI("POWER_LED", "power_led_adc", pwr_led, "ADC", "", "2800", "0", "4095");

  if (pwr_led >= 2800) {
    testTracking.failedTestName = "PWR LED";
    Serial.println("FAILED: Power LED not detected");
  } else {
    Serial.println("PASSED: Power LED OK");
  }

  return true;  // Test complete
}

// Non-blocking serial buffer update
// Call this regularly to read available serial data into the buffer
void updateSerialBuffer() {
  const size_t bufferSize = sizeof(testTracking.serialBuffer);

  while (Serial.available() > 0) {
    char c = Serial.read();

    // Add character to buffer if there's room
    if (testTracking.serialBufferPos < bufferSize - 1) {
      testTracking.serialBuffer[testTracking.serialBufferPos++] = c;
      testTracking.serialBuffer[testTracking.serialBufferPos] = '\0';  // Null terminate
    } else {
      // Buffer full - shift left by half to make room
      const size_t halfSize = bufferSize / 2;
      memmove(testTracking.serialBuffer,
              testTracking.serialBuffer + halfSize,
              halfSize);
      testTracking.serialBufferPos = halfSize;
      // Ensure we don't write past buffer
      if (testTracking.serialBufferPos < bufferSize - 1) {
        testTracking.serialBuffer[testTracking.serialBufferPos++] = c;
        testTracking.serialBuffer[testTracking.serialBufferPos] = '\0';
      }
    }
  }

  // Safety: ensure buffer is always null-terminated
  testTracking.serialBuffer[bufferSize - 1] = '\0';
}

// Check if a specific command is present in the serial buffer
// If found, removes it from the buffer and returns true
bool checkForSerialCommand(const char* command) {
  const size_t bufferSize = sizeof(testTracking.serialBuffer);

  // Ensure buffer is null-terminated before searching
  testTracking.serialBuffer[bufferSize - 1] = '\0';

  // Search for command in buffer
  char* found = strstr(testTracking.serialBuffer, command);
  if (found != nullptr) {
    // Command found! Calculate position after the command
    size_t commandLen = strlen(command);
    char* afterCommand = found + commandLen;

    // Use strnlen to safely get remaining length
    size_t maxRemaining = bufferSize - (afterCommand - testTracking.serialBuffer);
    size_t remainingLen = strnlen(afterCommand, maxRemaining);

    // Shift remaining data to start of buffer (safe because remainingLen is bounded)
    if (remainingLen > 0) {
      memmove(testTracking.serialBuffer, afterCommand, remainingLen);
    }
    testTracking.serialBuffer[remainingLen] = '\0';
    testTracking.serialBufferPos = remainingLen;

    Serial.print("Received command: ");
    Serial.println(command);
    return true;
  }
  return false;
}

// Clear the serial buffer
void clearSerialBuffer() {
  memset(testTracking.serialBuffer, 0, sizeof(testTracking.serialBuffer));
  testTracking.serialBufferPos = 0;
}

// ----------------------------------------------------------------------------
// I2C bootloader install, shared by both firmware phases
// ----------------------------------------------------------------------------

// One application image the jig can install through the I2C bootloader.
struct BootloadImage {
  const uint8_t* data;
  uint32_t size;
  uint16_t crc16;
  uint16_t fwVersion;       // REG_FW_VERSION the image must report once running
  const char* failPrefix;   // failedTestName prefix, so the two installs are told apart
};

const BootloadImage OLD_APP = {FADER_OLD_APP_IMAGE, FADER_OLD_APP_IMAGE_SIZE,
                               FADER_OLD_APP_IMAGE_CRC16, OLD_FW_VERSION_FOR_TEST, "FW OLD"};
const BootloadImage CURRENT_APP = {FADER_APP_IMAGE, FADER_APP_IMAGE_SIZE,
                                   FADER_APP_IMAGE_CRC16, FADER_APP_FW_VERSION, "FW BL"};

enum BootloadResult { BOOTLOAD_RUNNING, BOOTLOAD_DONE, BOOTLOAD_FAILED };

// True while an install is in progress in the current TestState, which is when
// the display shows page progress and loop() skips its idle delay.
bool bootloadActive() {
  if (testTracking.bootloadTotalPages == 0) return false;
  if (currentTestState == TEST_FW_I2C_UPDATE) return true;
  return currentTestState == TEST_FW_BOOTSTRAP &&
         testTracking.firmwarePhase == TestTracking::FIRMWARE_PHASE_INSTALL_OLD;
}

// The install is faderbuddy::FirmwareUpdater (hostLib/), the same sequence the
// ESPHome component runs - including writing the reset-vector page last, only
// after the rest of the image has verified, so a board that fails part-way
// stays in its bootloader rather than booting a half-written app.
//
// It runs in its own task on core 0, alongside loop() on core 1. Running it
// from loop() meant either pacing the transfer by the loop's display push and
// delay (~65ms a page, against ~8ms of bus time) or starving the display and
// LED breathing for whole slices at a time. Nothing else touches
// WireFaderBuddy while an install is running - the display is SPI and the
// INA3221 is on the other I2C peripheral - so the two run without contending.
//
// The task publishes completion through installState; the outcome and page
// progress are read from faderUpdater.
enum InstallState : uint8_t { INSTALL_IDLE, INSTALL_BUSY, INSTALL_DONE };
volatile InstallState installState = INSTALL_IDLE;
volatile bool installCancel = false;
const BootloadImage* installImage = nullptr;
TaskHandle_t installTaskHandle = nullptr;

void bootloadInstallTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    faderbuddy::FirmwareImage image = {installImage->data, installImage->size, installImage->crc16,
                                       installImage->fwVersion};
    faderbuddy::FirmwareUpdater::Options options;
    // The jig exists to exercise the install, so never skip it.
    options.skip_if_current = false;
    // The old app is an unversioned (FW_VERSION 0) build of firmware that does
    // support bootloader entry.
    options.require_bootloader_entry_version = false;
    // Ride out the bootloader's flash-write stalls as generously as this jig
    // always has.
    options.retry_attempts = 40;
    options.retry_delay_ms = 5;
    faderUpdater.start(image, options);
    while (faderUpdater.tick()) {
      if (installCancel) {
        faderUpdater.cancel();
        break;
      }
      // Page streaming already yields once per slice; the other stages are
      // waiting on the target, so don't hammer it.
      vTaskDelay(pdMS_TO_TICKS(faderUpdater.stage() == faderbuddy::UpdateStage::WRITE ? 1 : 5));
    }
    __atomic_store_n(&installState, INSTALL_DONE, __ATOMIC_RELEASE);
  }
}

void startBootloadStreamTask() {
  xTaskCreatePinnedToCore(bootloadInstallTask, "bl_install", 6144, nullptr, 1, &installTaskHandle, 0);
}

// Stop any install in flight and return the task to idle. A board pulled
// mid-install fails its next transaction on the Wire timeout, so this is quick
// even then; the wait is only a backstop.
void stopBootloadStream() {
  if (__atomic_load_n(&installState, __ATOMIC_ACQUIRE) == INSTALL_BUSY) {
    installCancel = true;
    uint32_t start = millis();
    while (__atomic_load_n(&installState, __ATOMIC_ACQUIRE) == INSTALL_BUSY && millis() - start < 3000) {
      delay(1);
    }
  }
  installCancel = false;
  installState = INSTALL_IDLE;
}

// Arm stepBootload() for a fresh install.
void resetBootload() {
  stopBootloadStream();
  testTracking.bootloadStarted = false;
  testTracking.bootloadPageIndex = 0;
  testTracking.bootloadTotalPages = 0;
}

// failedTestName suffix for each way an install can fail. These are what the
// jig has always reported, so test logs stay comparable.
const char* bootloadFailSuffix() {
  using faderbuddy::UpdateError;
  switch (faderUpdater.error()) {
    case UpdateError::NONE: return " ?";
    case UpdateError::BAD_IMAGE: return " IMG";
    case UpdateError::NOT_RESPONDING:
    case UpdateError::PREDATES_BOOTLOADER_ENTRY:
    case UpdateError::ENTER_FAILED:
    case UpdateError::NO_MARKER: return " ENTER";
    case UpdateError::NO_STATUS: return " STAT";
    case UpdateError::ERASE_FAILED: return " ERASE";
    case UpdateError::NVM_ERROR:
      return faderUpdater.failed_stage() == faderbuddy::UpdateStage::ERASE_WAIT ? " ERASE" : " WRITE";
    case UpdateError::WRITE_FAILED: return " WRITE";
    case UpdateError::CRC_READ_FAILED: return " CRCRD";
    case UpdateError::CRC_MISMATCH: return " CRC";
    case UpdateError::RUN_FAILED: return " RUN";
    case UpdateError::APP_DID_NOT_START: return " BOOT";
    case UpdateError::FW_VERSION_READ_FAILED: return " FWVER";
    case UpdateError::FW_VERSION_MISMATCH: return " NEWVER";
    case UpdateError::CANCELLED: return " ABORT";
  }
  return " ?";
}

// Non-blocking install of one image: enters the bootloader (from a running app,
// or finds it already resident), erases, streams, verifies on the target, runs
// the app, and confirms it reports the image's FW_VERSION. Call repeatedly
// until it stops returning BOOTLOAD_RUNNING; on BOOTLOAD_FAILED,
// testTracking.failedTestName says why.
BootloadResult stepBootload(const BootloadImage& img) {
  if (!testTracking.bootloadStarted) {
    testTracking.bootloadStarted = true;
    testTracking.bootloadStartTime = millis();
    testTracking.bootloadPageIndex = 0;
    testTracking.bootloadTotalPages = img.size / BL_PAGE_SIZE;
    Serial.printf("Installing FW_VERSION=%u over the I2C bootloader...\n", img.fwVersion);
    installImage = &img;
    installCancel = false;
    __atomic_store_n(&installState, INSTALL_BUSY, __ATOMIC_RELEASE);
    xTaskNotifyGive(installTaskHandle);
    return BOOTLOAD_RUNNING;
  }

  testTracking.bootloadPageIndex = faderUpdater.pages_written();

  if (__atomic_load_n(&installState, __ATOMIC_ACQUIRE) == INSTALL_BUSY) {
    // Overall watchdog for the whole sequence (generous margin over the
    // hardware-validated ~few-second full update).
    if (millis() - testTracking.bootloadStartTime > 20000) {
      Serial.println("FAILED: I2C bootloader install timed out");
      stopBootloadStream();
      testTracking.failedTestName = String(img.failPrefix) + " TMO";
      return BOOTLOAD_FAILED;
    }
    return BOOTLOAD_RUNNING;
  }
  installState = INSTALL_IDLE;

  if (!faderUpdater.succeeded()) {
    Serial.printf("FAILED: I2C bootloader install failed while %s: %s (wrote %u/%u pages)\n",
                  faderbuddy::update_stage_name(faderUpdater.failed_stage()), faderUpdater.error_message(),
                  faderUpdater.pages_written(), faderUpdater.pages_total());
    testTracking.failedTestName = String(img.failPrefix) + bootloadFailSuffix();
    return BOOTLOAD_FAILED;
  }
  Serial.printf("  wrote %u/%u pages\n", faderUpdater.pages_written(), faderUpdater.pages_total());
  Serial.printf("PASSED: I2C bootloader install of FW_VERSION=%u succeeded in %lu ms\n", img.fwVersion,
                (unsigned long)(millis() - testTracking.bootloadStartTime));
  return BOOTLOAD_DONE;
}

// TEST_FW_BOOTSTRAP: UPDI-flash the current bootloader and its fuses (via
// test_host.py), confirm it comes up resident with no application, then
// install the fixed old application through it over I2C. That leaves "a board
// running an old application", the precondition for testFwI2cUpdate() below,
// which drives the in-field-style update from the running app.
//
// No application goes over UPDI: serial UPDI is bound by USB round-trip latency
// per flash page, so flashing the old app that way made this phase ~25s.
bool testFwBootstrap() {
  // Initialize on first call
  if (testTracking.firmwarePhase == TestTracking::FIRMWARE_PHASE_PING &&
      testTracking.firmwarePhaseStartTime == 0) {
    testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_PING;
    testTracking.firmwarePhaseStartTime = millis();
    testTracking.firmwareCommandSent = false;
    clearSerialBuffer();  // Clear buffer
    Serial.println("=== Starting firmware bootstrap (bootloader UPDI flash + old app over I2C) ===");
  }

  // Non-blocking: read any available serial data into buffer
  updateSerialBuffer();

  switch (testTracking.firmwarePhase) {
    case TestTracking::FIRMWARE_PHASE_PING:
      if (!testTracking.firmwareCommandSent) {
        // Send ping command
        Serial.println(">>PING<<");
        testTracking.firmwareCommandSent = true;
        testTracking.firmwarePhaseStartTime = millis();
      }

      // Check for ACK (non-blocking)
      if (checkForSerialCommand(">>ACK<<")) {
        // ACK received, move to upload phase
        Serial.println("Host script acknowledged, requesting firmware upload");
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_UPLOAD;
        testTracking.firmwareCommandSent = false;
        testTracking.firmwarePhaseStartTime = millis();
      } else if (millis() - testTracking.firmwarePhaseStartTime > 2000) {
        // Timeout
        testTracking.failedTestName = "FW NO HOST";
        Serial.println("FAILED: No response from host script (is test_host.py running?)");
        return true;  // Test complete (failed)
      }
      return false;  // Still waiting

    case TestTracking::FIRMWARE_PHASE_UPLOAD:
      if (!testTracking.firmwareCommandSent) {
        // Send upload command
        Serial.println(">>START_FIRMWARE_UPLOAD<<");
        testTracking.firmwareCommandSent = true;
        testTracking.firmwarePhaseStartTime = millis();
      }

      // Check for SUCCESS or FAILURE (non-blocking)
      if (checkForSerialCommand(">>SUCCESS<<")) {
        Serial.println("PASSED: Bootloader UPDI flash succeeded");
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_WAIT_BOOTLOADER;
        testTracking.firmwareCommandSent = false;
        testTracking.firmwarePhaseStartTime = millis();
      } else if (checkForSerialCommand(">>FAILURE<<")) {
        // Upload failed
        testTracking.failedTestName = "FW UPLOAD";
        Serial.println("FAILED: Firmware upload failed");
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
        return true;  // Test complete (failed)
      } else if (millis() - testTracking.firmwarePhaseStartTime > 40000) {
        // Timeout (UPDI chip-erase + bootloader write + two fuse writes)
        testTracking.failedTestName = "FW TIMEOUT";
        Serial.println("FAILED: Firmware upload timed out");
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
        return true;  // Test complete (failed)
      }
      return false;  // Still waiting

    case TestTracking::FIRMWARE_PHASE_WAIT_BOOTLOADER:
      {
        // The chip was just erased and given only a bootloader, so it must come
        // up resident and report that it has no application. Anything else means
        // the erase or the fuses didn't take.
        uint8_t v;
        faderbuddy::BootloaderStatus bl;
        if (faderBuddy.read_probe(v) && v == BL_VERSION_MARKER && faderBuddy.bl_get_status(bl, 40, 5)) {
          uint8_t bv = bl.version, st = bl.status, le = bl.last_error;
          if (st != BL_STATUS_NO_APP) {
            testTracking.failedTestName = "FW BL HASAPP";
            Serial.printf("FAILED: fresh bootloader reports status=%u, expected NO_APP\n", st);
            testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
            return true;  // Test complete (failed)
          }
          Serial.printf("Bootloader resident with no app: ver=%u last_error=%u\n", bv, le);
          resetBootload();
          testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_INSTALL_OLD;
          testTracking.firmwarePhaseStartTime = millis();
          return false;
        }
        if (millis() - testTracking.firmwarePhaseStartTime > 2000) {
          testTracking.failedTestName = "FW BL I2C";
          Serial.println("FAILED: bootloader not answering over I2C after UPDI flash");
          testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
          return true;  // Test complete (failed)
        }
        return false;  // Still waiting for the bootloader to come up
      }

    case TestTracking::FIRMWARE_PHASE_INSTALL_OLD:
      switch (stepBootload(OLD_APP)) {
        case BOOTLOAD_RUNNING:
          return false;
        case BOOTLOAD_FAILED:
          testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
          return true;  // Test complete (failed)
        case BOOTLOAD_DONE:
          // Prime the phase for testFwI2cUpdate() and signal this TestState
          // (TEST_FW_BOOTSTRAP) is complete -- the outer state machine moves
          // on to TEST_FW_I2C_UPDATE.
          resetBootload();
          testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_BOOTLOAD_UPDATE;
          testTracking.firmwareCommandSent = false;
          testTracking.firmwarePhaseStartTime = millis();
          return true;  // Test complete (passed)
      }
      return false;

    case TestTracking::FIRMWARE_PHASE_COMPLETE:
      return true;  // Already complete

    default:
      return true;
  }
}

// TEST_FW_I2C_UPDATE: drive the I2C bootloader (entered from the running old
// app installed by testFwBootstrap()) to install and run the current
// application, then read the serial number back for the host.
bool testFwI2cUpdate() {
  // Non-blocking: read any available serial data into buffer (test_host.py
  // isn't involved in this phase, but keep the buffer drained regardless).
  updateSerialBuffer();

  switch (testTracking.firmwarePhase) {
    case TestTracking::FIRMWARE_PHASE_BOOTLOAD_UPDATE:
      switch (stepBootload(CURRENT_APP)) {
        case BOOTLOAD_RUNNING:
          return false;
        case BOOTLOAD_FAILED:
          testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
          return true;
        case BOOTLOAD_DONE:
          break;
      }
      reportDataI("FW_I2C_UPDATE", "pages_written", testTracking.bootloadTotalPages,
                  "pages", "", "");
      {
        char crcHex[8];
        snprintf(crcHex, sizeof(crcHex), "0x%04X", FADER_APP_IMAGE_CRC16);
        reportDataStr("FW_I2C_UPDATE", "image_crc16", crcHex);
        char ver[12];
        snprintf(ver, sizeof(ver), "%u.%u", FADER_APP_FW_VERSION >> 8, FADER_APP_FW_VERSION & 0xFF);
        reportDataStr("FW_I2C_UPDATE", "version_after", ver);
        // Report the version the DUT reported back (stepBootload() confirmed it
        // equals FADER_APP_FW_VERSION) so the host can log what shipped on this
        // board. REG_FW_VERSION is packed (major << 8) | minor.
        Serial.printf(">>FW_VERSION:%s<<\n", ver);
      }
      testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_READ_SERIAL;
      testTracking.firmwarePhaseStartTime = millis();
      return false;

    case TestTracking::FIRMWARE_PHASE_READ_SERIAL:
      {
        // Read serial number from motor fader and report to host
        uint8_t serial[10];
        if (faderBuddy.read_serial_number(serial)) {
          // Format as hex string and send to host
          Serial.print(">>SERIAL:");
          for (int i = 0; i < 10; i++) {
            if (serial[i] < 0x10) Serial.print("0");
            Serial.print(serial[i], HEX);
          }
          Serial.println("<<");

          testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
          return true;  // Test complete (passed)
        } else {
          // Failed to read serial number
          if (millis() - testTracking.firmwarePhaseStartTime > 2000) {
            // Timeout after 2 seconds
            testTracking.failedTestName = "FW SERIAL";
            Serial.println("FAILED: Could not read serial number from motor fader");
            testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
            return true;  // Test complete (failed)
          }
        }
        return false;  // Still trying
      }

    case TestTracking::FIRMWARE_PHASE_COMPLETE:
      return true;  // Already complete

    default:
      return true;
  }
}

bool testDebugLED() {
  const uint16_t THRESHOLD = 2800;
  const uint16_t HYSTERESIS = 100;
  const uint16_t THRESHOLD_HIGH = THRESHOLD + HYSTERESIS;  // 2900
  const uint16_t THRESHOLD_LOW = THRESHOLD - HYSTERESIS;   // 2700
  const uint32_t TEST_DURATION_MS = 512*5/2;
  const uint8_t MIN_TRANSITIONS = 4;
  const uint8_t MAX_TRANSITIONS = 6;

  // Initialize on first call
  if (testTracking.debugLedTestStartTime == 0) {
    testTracking.debugLedTestStartTime = millis();
    testTracking.debugLedTransitionCount = 0;

    // Read initial LED state
    uint16_t dbg_led = analogRead(PIN_PHOTODIODE_DBG);
    testTracking.debugLedState = (dbg_led < THRESHOLD);  // true if LED is ON (low reading)

    Serial.println("Testing debug LED blink pattern (2 seconds)...");
    Serial.print("Initial LED reading: ");
    Serial.print(dbg_led);
    Serial.print(", state: ");
    Serial.println(testTracking.debugLedState ? "ON" : "OFF");
  }

  // Read current LED value
  uint16_t dbg_led = analogRead(PIN_PHOTODIODE_DBG);

  // Check for transitions with hysteresis
  if (testTracking.debugLedState) {
    // LED is currently ON (low value), check for transition to OFF (high value)
    if (dbg_led >= THRESHOLD_HIGH) {
      testTracking.debugLedState = false;  // LED is now OFF
      testTracking.debugLedTransitionCount++;
      Serial.print("LED OFF transition #");
      Serial.print(testTracking.debugLedTransitionCount);
      Serial.print(" (value: ");
      Serial.print(dbg_led);
      Serial.println(")");
    }
  } else {
    // LED is currently OFF (high value), check for transition to ON (low value)
    if (dbg_led <= THRESHOLD_LOW) {
      testTracking.debugLedState = true;  // LED is now ON
      testTracking.debugLedTransitionCount++;
      Serial.print("LED ON transition #");
      Serial.print(testTracking.debugLedTransitionCount);
      Serial.print(" (value: ");
      Serial.print(dbg_led);
      Serial.println(")");
    }
  }

  // Check if test duration has elapsed
  if (millis() - testTracking.debugLedTestStartTime < TEST_DURATION_MS) {
    return false;  // Still running
  }

  // Test complete - check if transition count is in valid range
  Serial.print("Debug LED test complete. Transitions: ");
  Serial.println(testTracking.debugLedTransitionCount);

  reportDataI("DEBUG_LED", "led_transitions", testTracking.debugLedTransitionCount,
              "", "4", "6", "0", "10");

  if (testTracking.debugLedTransitionCount < MIN_TRANSITIONS) {
    testTracking.failedTestName = "DBG LED FEW";
    Serial.print("FAILED: Too few transitions (");
    Serial.print(testTracking.debugLedTransitionCount);
    Serial.print(" < ");
    Serial.print(MIN_TRANSITIONS);
    Serial.println(")");
  } else if (testTracking.debugLedTransitionCount > MAX_TRANSITIONS) {
    testTracking.failedTestName = "DBG LED MANY";
    Serial.print("FAILED: Too many transitions (");
    Serial.print(testTracking.debugLedTransitionCount);
    Serial.print(" > ");
    Serial.print(MAX_TRANSITIONS);
    Serial.println(")");
  } else {
    Serial.println("PASSED: Debug LED blink pattern OK");
  }

  return true;  // Test complete
}

void readFaderBuddyVersionInfo() {
  versionInfo.valid = false;

  if (faderBuddy.read_probe(versionInfo.protocolVersion)) {
    Serial.print("Motor Fader Protocol Version: ");
    Serial.println(versionInfo.protocolVersion);
    // Reported to the host the same way FW_VERSION is, so the test report can
    // record what the board actually answered rather than a compiled-in guess.
    Serial.printf(">>PROTOCOL_VERSION:%u<<\n", versionInfo.protocolVersion);

    // Require a valid app protocol version (see i2c_data.h REG_VERSION: >= 5 is
    // an app; BL_VERSION_MARKER/0xFF mean bootloader-resident/no response).
    if (versionInfo.protocolVersion < 5) {
      Serial.print("ERROR: Expected protocol version >= 5, got ");
      Serial.println(versionInfo.protocolVersion);
      versionInfo.valid = false;
      return;
    }

    if (!faderBuddy.read_fw_version(versionInfo.fwVersion, 40, 5)) {
      Serial.println("Failed to read firmware version");
      return;
    }

    versionInfo.valid = true;
  } else {
    Serial.println("Failed to read protocol version");
  }
}

void readFaderBuddyState() {
  faderState.valid = false;

  uint32_t state;
  if (!faderBuddy.read_state(state)) {
    Serial.println("Failed to read motor fader state");
    return;
  }

  // Extract fields from state bitfield
  faderState.touch = (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp;
  faderState.mode = (state & STATE_MODE_bm) >> STATE_MODE_bp;
  faderState.position = (state & STATE_POSITION_bm) >> STATE_POSITION_bp;
  faderState.rawADC = (state & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp;

  // Read uptime separately
  if (!faderBuddy.read_uptime(faderState.uptime)) {
    Serial.println("Failed to read uptime");
    return;
  }

  // Read touch diagnostics
  if (!faderBuddy.read_touch_delta(faderState.touchDelta)) {
    Serial.println("Failed to read touch delta");
    return;
  }

  if (!faderBuddy.read_touch_reference(faderState.touchReference)) {
    Serial.println("Failed to read touch reference");
    return;
  }

  if (!faderBuddy.read_touch_recal_count(faderState.touchRecalCount)) {
    Serial.println("Failed to read touch recal count");
    return;
  }

  faderState.valid = true;
}

bool testSelfCalibration() {
  const uint32_t TOTAL_TIMEOUT_MS = 15000;
  const uint32_t MODE_ENTRY_TIMEOUT_MS = 1000;
  const uint16_t MIN_ADC_THRESHOLD = 100;
  const uint16_t MAX_ADC_THRESHOLD = 1900;

  // Initialize on first call
  if (testTracking.selfCalStartTime == 0) {
    testTracking.selfCalStartTime = millis();
    testTracking.selfCalRequestSent = false;
    testTracking.selfCalEnteredMode = false;
    testTracking.selfCalMinADC = 0xFFFF;  // Start with max value
    testTracking.selfCalMaxADC = 0;       // Start with min value
    // Self-calibration is the first time the motor is driven hard, so start the
    // peak-current window here; the movement checks report it.
    testTracking.peakMotorCurrentMa = 0;
    Serial.println("Starting self-calibration test (8 seconds max)...");
  }

  // Send self-calibration command on first call
  if (!testTracking.selfCalRequestSent) {
    if (faderBuddy.self_calibrate()) {
      Serial.println("Self-calibration command sent");
      testTracking.selfCalRequestSent = true;
    } else {
      testTracking.failedTestName = "CAL CMD";
      Serial.println("FAILED: Could not send self-calibration command");
      return true;  // Test complete (failed)
    }
  }

  // Read motor fader state continuously
  readFaderBuddyState();

  if (!faderState.valid) {
    // Can't read state, check for overall timeout
    if (millis() - testTracking.selfCalStartTime > TOTAL_TIMEOUT_MS) {
      testTracking.failedTestName = "CAL NO I2C";
      Serial.println("FAILED: Cannot read motor fader state");
      return true;  // Test complete (failed)
    }
    return false;  // Keep trying
  }

  // Update min/max ADC values
  if (faderState.rawADC < testTracking.selfCalMinADC) {
    testTracking.selfCalMinADC = faderState.rawADC;
    Serial.print("Self-cal: New min ADC = ");
    Serial.println(faderState.rawADC);
  }
  if (faderState.rawADC > testTracking.selfCalMaxADC) {
    testTracking.selfCalMaxADC = faderState.rawADC;
    Serial.print("Self-cal: New max ADC = ");
    Serial.println(faderState.rawADC);
  }

  // Check if mode entered self-calibration
  if (!testTracking.selfCalEnteredMode) {
    if (faderState.mode == MODE_SELF_CALIBRATION) {
      testTracking.selfCalEnteredMode = true;
      Serial.println("Self-calibration mode entered");
    } else if (millis() - testTracking.selfCalStartTime > MODE_ENTRY_TIMEOUT_MS) {
      testTracking.failedTestName = "CAL NO ENTRY";
      Serial.print("FAILED: Did not enter calibration mode (current mode: ");
      Serial.print(getModeString(faderState.mode));
      Serial.println(")");
      return true;  // Test complete (failed)
    }
  }

  // Check for ERROR mode
  if (faderState.mode == MODE_ERROR) {
    testTracking.failedTestName = "CAL ERROR";
    Serial.println("FAILED: Motor fader entered ERROR mode during calibration");
    return true;  // Test complete (failed)
  }

  // Check for calibration completion (transition away from MODE_SELF_CALIBRATION)
  if (testTracking.selfCalEnteredMode && faderState.mode == MODE_INPUT_IDLE) {
    Serial.print("Self-calibration completed. Mode: ");
    Serial.println(getModeString(faderState.mode));
    Serial.print("ADC range observed: ");
    Serial.print(testTracking.selfCalMinADC);
    Serial.print(" - ");
    Serial.println(testTracking.selfCalMaxADC);

    reportDataI("SELF_CALIBRATION", "travel_min_adc", testTracking.selfCalMinADC,
                "ADC", "", "100", "0", "2047");
    reportDataI("SELF_CALIBRATION", "travel_max_adc", testTracking.selfCalMaxADC,
                "ADC", "1900", "", "0", "2047");
    reportDataI("SELF_CALIBRATION", "travel_span_adc",
                (long)testTracking.selfCalMaxADC - (long)testTracking.selfCalMinADC,
                "ADC", "", "", "0", "2047");

    // Validate ADC extremes were reached
    if (testTracking.selfCalMinADC > MIN_ADC_THRESHOLD) {
      testTracking.failedTestName = "CAL ADC MIN";
      Serial.print("FAILED: ADC min (");
      Serial.print(testTracking.selfCalMinADC);
      Serial.print(") did not reach below ");
      Serial.println(MIN_ADC_THRESHOLD);
      return true;  // Test complete (failed)
    }

    if (testTracking.selfCalMaxADC < MAX_ADC_THRESHOLD) {
      testTracking.failedTestName = "CAL ADC MAX";
      Serial.print("FAILED: ADC max (");
      Serial.print(testTracking.selfCalMaxADC);
      Serial.print(") did not reach above ");
      Serial.println(MAX_ADC_THRESHOLD);
      return true;  // Test complete (failed)
    }

    Serial.println("PASSED: Self-calibration completed successfully");
    // The characterisation is only settled once self-calibration has finished,
    // so this is the first point it is worth reading.
    reportMotorCal();
    return true;  // Test complete (passed)
  }

  // Check for overall timeout
  if (millis() - testTracking.selfCalStartTime > TOTAL_TIMEOUT_MS) {
    testTracking.failedTestName = "CAL TIMEOUT";
    Serial.println("FAILED: Self-calibration timed out");
    return true;  // Test complete (failed)
  }

  return false;  // Still running
}

// Record how long a commanded move took to settle and how far off it ended up.
// `slot` indexes testTracking.settleMs/settleError; `commanded` is the position
// that was written, so the residual is measured rather than merely bounded.
void recordSettle(uint8_t slot, uint8_t commanded) {
  uint32_t elapsed = millis() - testTracking.testStartTime;
  testTracking.settleMs[slot] = (elapsed > 0xFFFF) ? 0xFFFF : (uint16_t)elapsed;
  int16_t err = (int16_t)faderState.position - (int16_t)commanded;
  if (err < 0) err = -err;
  testTracking.settleError[slot] = (err > 255) ? 255 : (uint8_t)err;
  Serial.printf("Reached idle state (%lu ms, off by %u)\n",
                (unsigned long)elapsed, testTracking.settleError[slot]);
}

bool testDiagnostics() {
  const uint32_t MOVEMENT_TIMEOUT_MS = 3000;

  // Initialize on first call
  if (testTracking.diagnosticsMovementStep == 0) {
    Serial.println("Starting I2C diagnostics...");
    readFaderBuddyVersionInfo();
  }

  // Continuously read motor state
  readFaderBuddyState();

  // State machine for movements:
  // Step 0: Command position 10
  // Step 1: Wait for idle
  // Step 2: Command position 200
  // Step 3: Wait for idle
  // Step 4: Command position 80
  // Step 5: Wait for idle
  // Step 6: Complete

  switch (testTracking.diagnosticsMovementStep) {
    case 0:  // Command position 10
      faderBuddy.write_layer_target(0, 10);
      Serial.println("Commanding position 10");
      testTracking.testStartTime = millis();  // Start timeout for idle wait
      testTracking.diagnosticsMovementStep = 1;
      return false;

    case 1:  // Wait for idle after position 10
      if (faderState.valid && faderState.mode == MODE_INPUT_IDLE) {
        recordSettle(0, 10);
        testTracking.diagnosticsMovementStep = 2;
        return false;
      }
      if (millis() - testTracking.testStartTime > MOVEMENT_TIMEOUT_MS) {
        testTracking.failedTestName = "DIAG MVT1";
        Serial.println("FAILED: Movement 1 timed out (did not reach idle within 3s)");
        return true;
      }
      return false;

    case 2:  // Command position 200
      faderBuddy.write_layer_target(0, 200);
      Serial.println("Commanding position 200");
      testTracking.testStartTime = millis();  // Start timeout for idle wait
      testTracking.diagnosticsMovementStep = 3;
      return false;

    case 3:  // Wait for idle after position 200
      if (faderState.valid && faderState.mode == MODE_INPUT_IDLE) {
        recordSettle(1, 200);
        testTracking.diagnosticsMovementStep = 4;
        return false;
      }
      if (millis() - testTracking.testStartTime > MOVEMENT_TIMEOUT_MS) {
        testTracking.failedTestName = "DIAG MVT2";
        Serial.println("FAILED: Movement 2 timed out (did not reach idle within 3s)");
        return true;
      }
      return false;

    case 4:  // Command position 80
      faderBuddy.write_layer_target(0, 80);
      Serial.println("Commanding position 80");
      testTracking.testStartTime = millis();  // Start timeout for idle wait
      testTracking.diagnosticsMovementStep = 5;
      return false;

    case 5:  // Wait for idle after position 80
      if (faderState.valid && faderState.mode == MODE_INPUT_IDLE) {
        recordSettle(2, 80);
        testTracking.diagnosticsMovementStep = 6;
        return false;
      }
      if (millis() - testTracking.testStartTime > MOVEMENT_TIMEOUT_MS) {
        testTracking.failedTestName = "DIAG MVT3";
        Serial.println("FAILED: Movement 3 timed out (did not reach idle within 3s)");
        return true;
      }
      return false;

    case 6:  // Complete
      for (uint8_t i = 0; i < 3; i++) {
        char key[20];
        snprintf(key, sizeof(key), "settle_ms_%u", i + 1);
        reportDataI("MOVEMENT", key, testTracking.settleMs[i], "ms", "", "3000", "0", "3300");
      }
      {
        uint8_t worst = 0;
        for (uint8_t i = 0; i < 3; i++) {
          if (testTracking.settleError[i] > worst) worst = testTracking.settleError[i];
        }
        reportDataI("MOVEMENT", "worst_settle_error", worst, "/255", "", "");
      }
      // Covers self-calibration and these movements - the whole heavy-driving
      // window. No limit yet: this is a baseline to gather before setting one.
      reportDataF("MOVEMENT", "peak_motor_current", testTracking.peakMotorCurrentMa, 1,
                  "mA", "", "");
      Serial.println("PASSED: Diagnostics complete");
      return true;

    default:
      return true;
  }
}

bool testTouchSensor() {
  // Initialize on first call
  if (testTracking.touchTestPhase == TestTracking::TOUCH_PHASE_CHECK_NO_TOUCH &&
      testTracking.touchTestPhaseStartTime == 0) {
    testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CHECK_NO_TOUCH;
    testTracking.touchTestPhaseStartTime = millis();
    Serial.println("Starting touch sensor test...");

    // Re-attach servo for test (it was detached in setup)
    // servo.attach(PIN_SERVO);
    // servo.write(SERVO_CLEAR_POS);
  }

  // Continuously read motor fader state throughout the test
  readFaderBuddyState();

  if (!faderState.valid) {
    Serial.println("Failed to read motor state during touch test");
    return false;  // Keep trying
  }

  switch (testTracking.touchTestPhase) {
    case TestTracking::TOUCH_PHASE_CHECK_NO_TOUCH:
      // Confirm touch is not detected before starting
      if (faderState.touch) {
        testTracking.failedTestName = "TCH INITIAL";
        Serial.println("FAILED: Touch detected at start of test");
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }
      Serial.println("Touch not detected, proceeding to movement");
      testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_MOVE_TO_POSITION;
      testTracking.touchTestPhaseStartTime = millis();
      return false;

    case TestTracking::TOUCH_PHASE_MOVE_TO_POSITION:
      // Command fader to position 0
      if (millis() - testTracking.touchTestPhaseStartTime < 100) {
        // Give a small delay before commanding to ensure we're ready
        return false;
      }
      if (!faderBuddy.write_layer_target(0, 42)) {
        testTracking.failedTestName = "TCH CMD POS";
        Serial.println("FAILED: Could not command position 0");
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }
      Serial.println("Commanded position 0, waiting for idle");
      testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_WAIT_FOR_IDLE;
      testTracking.touchTestPhaseStartTime = millis();
      return false;

    case TestTracking::TOUCH_PHASE_WAIT_FOR_IDLE:
      // Wait for fader to report idle state (timeout 9 seconds)
      if (faderState.mode == MODE_ERROR) {
        testTracking.failedTestName = "TCH ERROR";
        Serial.println("FAILED: Fader entered error state during movement");
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }

      if (faderState.mode == MODE_INPUT_IDLE) {
        Serial.println("Fader reached idle state");
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_SERVO_TOUCH;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }

      if (millis() - testTracking.touchTestPhaseStartTime > 9000) {
        testTracking.failedTestName = "TCH IDLE TMO";
        Serial.print("FAILED: Fader did not reach idle within 9 seconds (mode: ");
        Serial.print(getModeString(faderState.mode));
        Serial.println(")");
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }
      return false;  // Still waiting

    case TestTracking::TOUCH_PHASE_SERVO_TOUCH:
      // Activate servo to touch position
      servo.write(SERVO_TOUCH_POS);
      Serial.println("Servo activated to touch position");
      testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_WAIT_FOR_TOUCH_DETECT;
      testTracking.touchTestPhaseStartTime = millis();
      return false;

    case TestTracking::TOUCH_PHASE_WAIT_FOR_TOUCH_DETECT:
      // Wait for touch to be detected (timeout 3 seconds)
      if (faderState.touch) {
        // The phase timer was started immediately after the servo was driven to
        // the touch position, so this is the actuation-to-detect latency.
        testTracking.touchDetectMs = (uint16_t)(millis() - testTracking.touchTestPhaseStartTime);
        testTracking.touchDeltaAtTouch = faderState.touchDelta;
        Serial.printf("Touch detected! (%u ms, delta %d)\n",
                      testTracking.touchDetectMs, testTracking.touchDeltaAtTouch);
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CONFIRM_TOUCH;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }

      if (millis() - testTracking.touchTestPhaseStartTime > 3000) {
        testTracking.failedTestName = "TCH NO DET";
        Serial.println("FAILED: Touch not detected within 3 seconds");
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }
      return false;  // Still waiting

    case TestTracking::TOUCH_PHASE_CONFIRM_TOUCH:
      // Wait 1 second and confirm touch is still detected
      if (millis() - testTracking.touchTestPhaseStartTime < 200) {
        // Debounce for a bit after initial touch
        return false;
      }
      if (millis() - testTracking.touchTestPhaseStartTime < 1000) {
        if (!faderState.touch) {
          testTracking.failedTestName = "TCH LOST";
          Serial.println("FAILED: Touch lost during confirmation period");
          testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE;
          testTracking.touchTestPhaseStartTime = millis();
          return false;
        }
        return false;  // Still confirming
      }

      Serial.println("Touch confirmed for 1 second");
      testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_SERVO_CLEAR;
      testTracking.touchTestPhaseStartTime = millis();
      return false;

    case TestTracking::TOUCH_PHASE_SERVO_CLEAR:
      // Move servo to clear position
      servo.write(SERVO_CLEAR_POS);
      Serial.println("Servo moved to clear position");
      testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_WAIT_FOR_TOUCH_CLEAR;
      testTracking.touchTestPhaseStartTime = millis();
      return false;

    case TestTracking::TOUCH_PHASE_WAIT_FOR_TOUCH_CLEAR:
      // Wait for touch to clear (timeout 3 seconds)
      if (!faderState.touch) {
        testTracking.touchReleaseMs = (uint16_t)(millis() - testTracking.touchTestPhaseStartTime);
        Serial.printf("Touch cleared (%u ms)\n", testTracking.touchReleaseMs);
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_FINAL_WAIT;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }

      if (millis() - testTracking.touchTestPhaseStartTime > 3000) {
        testTracking.failedTestName = "TCH NO CLR";
        Serial.println("FAILED: Touch did not clear within 3 seconds");
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE;
        testTracking.touchTestPhaseStartTime = millis();
        return false;
      }
      return false;  // Still waiting

    case TestTracking::TOUCH_PHASE_FINAL_WAIT:
      // // Wait 2 seconds after touch cleared
      // if (millis() - testTracking.touchTestPhaseStartTime < 2000) {
      //   return false;  // Still waiting
      // }

      // // Disable servo
      // servo.detach();
      // Serial.println("Servo disabled");
      reportDataI("TOUCH_SENSOR", "touch_reference", faderState.touchReference, "units", "", "");
      reportDataI("TOUCH_SENSOR", "touch_delta", testTracking.touchDeltaAtTouch, "units", "", "");
      reportDataI("TOUCH_SENSOR", "touch_detect_ms", testTracking.touchDetectMs, "ms", "", "3000", "0", "3300");
      reportDataI("TOUCH_SENSOR", "touch_release_ms", testTracking.touchReleaseMs, "ms", "", "3000", "0", "3300");
      reportDataI("TOUCH_SENSOR", "touch_recal_count", faderState.touchRecalCount, "", "", "");
      Serial.println("PASSED: Touch sensor test complete");
      return true;  // Test complete (passed)

    case TestTracking::TOUCH_PHASE_CLEANUP_ON_FAILURE:
      servo.write(SERVO_CLEAR_POS);
      Serial.println("Moving servo to clear position for cleanup");
      return true;  // Test complete (failed)
      // // On first entry to cleanup phase, move servo to clear and start timer
      // if (millis() - testTracking.touchTestPhaseStartTime < 100) {
      //   servo.write(SERVO_CLEAR_POS);
      //   Serial.println("Moving servo to clear position for cleanup");
      //   return false;
      // }

      // // Wait 2 seconds for servo to reach clear position
      // if (millis() - testTracking.touchTestPhaseStartTime < 2000) {
      //   return false;  // Still waiting
      // }

      // // Disable servo and complete test (failed)
      // servo.detach();
      // Serial.println("Servo disabled after failure");
      // return true;  // Test complete (failed)

    default:
      return true;
  }
}


void handleTestStateMachine(bool presencePressed, float v0, float i0, float v1, float i1) {
  bool presenceJustPressed = presencePressed && !lastPresenceState;
  bool presenceJustReleased = !presencePressed && lastPresenceState;

  // Abort tests if presence released during any active test
  if (presenceJustReleased &&
      currentTestState != TEST_IDLE &&
      currentTestState != TEST_PASSED &&
      currentTestState != TEST_FAILED) {
    Serial.println("\n=== Test Aborted (presence released) ===\n");

    // Ensure the servo isn't left resting on the fader (e.g. mid touch-sensor test)
    servo.write(SERVO_CLEAR_POS);

    // And that an I2C install's streaming task isn't left writing to the bus
    stopBootloadStream();

    // Report cancellation to host script for CSV logging
    Serial.print(">>TEST_RESULT:CANCELLED:");
    Serial.print(getTestStateName(currentTestState));
    Serial.println("<<");

    currentTestState = TEST_IDLE;
    lastReportedTestState = TEST_IDLE;
    clearSerialBuffer();  // Clear serial buffer
    versionInfo.valid = false;
    faderState.valid = false;
    lastPresenceState = presencePressed;
    return;
  }

  // Peak motor-rail draw, sampled only while the motor is actually driven.
  // Idle draw is already covered by the MOTOR_POWER check.
  if (currentTestState == TEST_SELF_CALIBRATION || currentTestState == TEST_DIAGNOSTICS ||
      currentTestState == TEST_TOUCH_SENSOR) {
    if (i1 > testTracking.peakMotorCurrentMa) {
      testTracking.peakMotorCurrentMa = i1;
    }
  }

  switch (currentTestState) {
    case TEST_IDLE:
      // Deliberately no I2C traffic here.
      //
      // Presence asserting leaves this state immediately, so anything polled in
      // idle is by definition aimed at a board that is NOT fully seated - i.e.
      // exactly while the connector is still mating. Hammering a DUT through
      // that window is what wedged the bus: a transaction interrupted by
      // contact bounce or a brownout leaves the DUT's TWI holding SDA low
      // mid-byte, waiting for clocks that never come, and every subsequent
      // transfer then burns the full Wire timeout until the board is pulled.
      //
      // A blank chip never drives SDA, which is why unprogrammed boards were
      // unaffected and boards with firmware on them hung.

      if (presenceJustPressed) {
        // Debounce delay before starting tests
        delay(50);
        Serial.println("\n=== Starting Test Sequence ===");
        // Reset test tracking
        testTracking.sampleCount = 0;
        testTracking.diagnosticsMovementStep = 0;
        testTracking.failedTestName = "";
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_PING;
        testTracking.firmwarePhaseStartTime = 0;
        testTracking.firmwareCommandSent = false;
        resetBootload();
        testTracking.debugLedTestStartTime = 0;
        testTracking.debugLedTransitionCount = 0;
        testTracking.debugLedState = false;
        testTracking.selfCalStartTime = 0;
        testTracking.selfCalRequestSent = false;
        testTracking.selfCalEnteredMode = false;
        testTracking.selfCalMinADC = 0xFFFF;
        testTracking.selfCalMaxADC = 0;
        testTracking.touchTestPhase = TestTracking::TOUCH_PHASE_CHECK_NO_TOUCH;
        testTracking.touchTestPhaseStartTime = 0;
        testTracking.phaseStartTime = 0;
        testTracking.peakMotorCurrentMa = 0;
        testTracking.touchDetectMs = 0;
        testTracking.touchReleaseMs = 0;
        testTracking.touchDeltaAtTouch = 0;
        for (uint8_t i = 0; i < 3; i++) {
          testTracking.settleMs[i] = 0;
          testTracking.settleError[i] = 0;
        }
        clearSerialBuffer();  // Clear serial buffer
        versionInfo.valid = false;
        faderState.valid = false;
        currentTestState = TEST_LOGIC_POWER;
      }
      break;

    case TEST_LOGIC_POWER:
      if (testLogicPower(v0, i0)) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          testTracking.sampleCount = 0;  // Reset for next test
          currentTestState = TEST_MOTOR_POWER;
        }
      }
      break;

    case TEST_MOTOR_POWER:
      if (testMotorPower(v1, i1)) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          currentTestState = TEST_POWER_LED;
        }
      }
      break;

    case TEST_POWER_LED:
      if (testPowerLED()) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          currentTestState = TEST_FW_BOOTSTRAP;
        }
      }
      break;

    case TEST_FW_BOOTSTRAP:
      // Dummy mode: skip both the bootstrap (needs test_host.py + UPDI) and
      // the I2C bootloader update (needs a real fader) after a simple delay,
      // so the rest of the jig flow can be exercised on the bench alone.
      if (dummyMode) {
        if (testTracking.firmwarePhaseStartTime == 0) {
          testTracking.firmwarePhaseStartTime = millis();
          Serial.println("DUMMY MODE: Skipping firmware bootstrap + I2C update (2s delay)");
        } else if (millis() - testTracking.firmwarePhaseStartTime >= 2000) {
          Serial.println("DUMMY MODE: Firmware bootstrap + I2C update skipped");
          currentTestState = TEST_DEBUG_LED;
        }
        break;
      }
      if (testFwBootstrap()) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          currentTestState = TEST_FW_I2C_UPDATE;
        }
      }
      break;

    case TEST_FW_I2C_UPDATE:
      if (testFwI2cUpdate()) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          currentTestState = TEST_DEBUG_LED;
        }
      }
      break;

    case TEST_DEBUG_LED:
      if (testDebugLED()) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          currentTestState = TEST_SELF_CALIBRATION;
        }
      }
      break;

    case TEST_SELF_CALIBRATION:
      if (testSelfCalibration()) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          currentTestState = TEST_DIAGNOSTICS;
        }
      }
      break;

    case TEST_DIAGNOSTICS:
      if (testDiagnostics()) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          currentTestState = TEST_TOUCH_SENSOR;
        }
      }
      break;

    case TEST_TOUCH_SENSOR:
      if (testTouchSensor()) {
        if (!testTracking.failedTestName.isEmpty()) {
          currentTestState = TEST_FAILED;
        } else {
          Serial.println("\n=== ALL TESTS PASSED ===\n");
          currentTestState = TEST_PASSED;
        }
      }
      break;

    case TEST_PASSED:
      // Only while the board is still seated: presence releases before the
      // contacts separate, so this stops before the removal bounce window.
      if (presencePressed) {
        readFaderBuddyState();
      }

      if (presenceJustReleased) {
        Serial.println("Returning to idle\n");
        currentTestState = TEST_IDLE;
        versionInfo.valid = false;
        faderState.valid = false;
      }
      break;

    case TEST_FAILED:
      // As TEST_PASSED: seated boards only.
      if (presencePressed) {
        readFaderBuddyState();
      }

      if (presenceJustReleased) {
        Serial.println("Test failed, returning to idle\n");
        currentTestState = TEST_IDLE;
        versionInfo.valid = false;
        faderState.valid = false;
      }
      break;
  }

  // Report test lifecycle transitions to the host script so it can log results to CSV.
  // Only the interesting transitions (test start / pass / fail) are sent; everything
  // else (e.g. moving between individual sub-tests) updates lastReportedTestState
  // without emitting anything.
  if (currentTestState != lastReportedTestState) {
    // Close out the phase being left before anything else, so the DATA lines a
    // test emits as it finishes fall inside its own PHASE_START/PHASE_END pair.
    const char* leaving = getTestPhaseKey(lastReportedTestState);
    if (leaving != nullptr) {
      reportPhaseEnd(leaving, currentTestState != TEST_FAILED,
                     millis() - testTracking.phaseStartTime);
    }

    if (currentTestState == TEST_LOGIC_POWER) {
      Serial.println(">>TEST_START<<");
    } else if (currentTestState == TEST_PASSED) {
      Serial.println(">>TEST_RESULT:PASS<<");
    } else if (currentTestState == TEST_FAILED) {
      Serial.print(">>TEST_RESULT:FAIL:");
      Serial.print(testTracking.failedTestName);
      Serial.println("<<");
    }

    const char* entering = getTestPhaseKey(currentTestState);
    if (entering != nullptr) {
      testTracking.phaseStartTime = millis();
      reportPhaseStart(entering);
    }
    lastReportedTestState = currentTestState;
  }

  lastPresenceState = presencePressed;
}


// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  // Left button (active low): press-and-hold toggles dummy mode.
  bool buttonPressed = (digitalRead(PIN_BUTTON_LEFT) == LOW);
  if (buttonPressed && !lastButtonPressed) {
    buttonPressStartTime = millis();
    buttonHoldHandled = false;
  } else if (buttonPressed && !buttonHoldHandled &&
             millis() - buttonPressStartTime >= BUTTON_HOLD_MS) {
    dummyMode = !dummyMode;
    buttonHoldHandled = true;
    Serial.printf("Dummy mode %s (button hold)\n", dummyMode ? "ENABLED" : "DISABLED");
  }
  lastButtonPressed = buttonPressed;

  bool pressed = !digitalRead(PIN_PRESENCE_SWITCH);

  // Read power measurements
  float voltage0 = ina3221.getBusVoltage(0);
  float current0 = ina3221.getCurrentAmps(0) * 1000;
  float voltage1 = ina3221.getBusVoltage(1);
  float current1 = ina3221.getCurrentAmps(1) * 1000;

  // Handle test state machine
  handleTestStateMachine(pressed, voltage0, current0, voltage1, current1);

  // LED control based on state
  if (currentTestState == TEST_IDLE) {
    analogWrite(PIN_LED_RED, 20);
    analogWrite(PIN_LED_GREEN, 5);
  } else if (currentTestState == TEST_PASSED) {
    // Solid green for passed
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_GREEN, 700);
  } else if (currentTestState == TEST_FAILED) {
    // Blinking red for failed
    analogWrite(PIN_LED_RED, (millis() % 350 < 175) * 500 + 500);
    analogWrite(PIN_LED_GREEN, 0);
  } else {
    // Alternating red/green breathing during tests (1s cycle, 30-60% brightness)
    float breathe = sin(millis() / 500.0 * 2.0 * PI);
    uint16_t brightnessRed = LERP(breathe, -1.0, 1.0, 0.1 * 1023, 0.5 * 1023);
    uint16_t brightnessGreen = LERP(-breathe, -1.0, 1.0, 0.03 * 1023, 0.15 * 1023);

    analogWrite(PIN_LED_RED, brightnessRed);
    analogWrite(PIN_LED_GREEN, brightnessGreen);
  }

  updateDisplay(voltage0, current0, voltage1, current1);

  delay(10);
}
