#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include <LittleFS.h>

#include "Adafruit_INA3221.h"
#include "motor_fader_i2c.h"

#define FlashFS LittleFS

#define PIN_LED_RED 21
#define PIN_LED_GREEN 22

#define PIN_PRESENCE_SWITCH 36

#define PIN_PHOTODIODE_PWR 37
#define PIN_PHOTODIODE_DBG 38


#define PIN_INA_SCL 13
#define PIN_INA_SDA 17

#define PIN_MOTOR_FADER_SCL 27
#define PIN_MOTOR_FADER_SDA 26

#define PIN_SERVO 32
#define SERVO_TOUCH_POS 85
#define SERVO_CLEAR_POS 50

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

TwoWire WireMotorFader = TwoWire(1);  // Use second I2C peripheral
MotorFaderI2C motorFader;

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
  TEST_FIRMWARE_INSTALL,  // Programming firmware
  TEST_DEBUG_LED,         // Testing debug LED blink pattern
  TEST_SELF_CALIBRATION,  // Self-calibration test
  TEST_DIAGNOSTICS,       // I2C diagnostics (10 seconds)
  TEST_PASSED,            // All tests passed
  TEST_FAILED             // One or more tests failed
};

TestState currentTestState = TEST_IDLE;
bool lastPresenceState = false;

// Motor fader version info (read once per test run)
struct MotorFaderVersion {
  uint8_t protocolVersion;
  bool valid;
} versionInfo = {0, false};

// Motor fader state (read continuously during diagnostics)
struct MotorFaderState {
  bool touch;
  uint8_t mode;
  uint8_t position;
  uint16_t rawADC;
  uint32_t uptime;
  int16_t touchDelta;
  uint16_t touchReference;
  uint16_t touchRecalCount;
  bool valid;
} motorState = {false, 0, 0, 0, 0, 0, 0, 0, false};

// Test tracking
struct TestTracking {
  uint32_t testStartTime;
  uint32_t diagnosticsStartTime;
  uint8_t diagnosticsMovementStep;

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
  uint16_t selfCalMinADC;
  uint16_t selfCalMaxADC;

  // Test results
  String failedTestName;
} testTracking = {0, 0, 0, 0, 0, 0, 0, 0, TestTracking::FIRMWARE_PHASE_PING, 0, false, {0}, 0, 0, 0, false, 0, false, false, 0xFFFF, 0, ""};

void setup() {
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_PRESENCE_SWITCH, INPUT);

  pinMode(PIN_PHOTODIODE_PWR, INPUT);
  pinMode(PIN_PHOTODIODE_DBG, INPUT);

  // Configure LED PWM for high resolution (10-bit = 1024 levels)
  analogWriteResolution(10);

  Serial.begin(115200);
  Wire.begin(PIN_INA_SDA, PIN_INA_SCL);

  // Initialize motor fader I2C on separate bus
  WireMotorFader.begin(PIN_MOTOR_FADER_SDA, PIN_MOTOR_FADER_SCL);
  motorFader.begin(&WireMotorFader);

  servo.attach(PIN_SERVO);

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

const String getTestStateName(TestState state) {
  switch (state) {
    case TEST_IDLE: return "IDLE";
    case TEST_LOGIC_POWER: return "1: LOGIC PWR";
    case TEST_MOTOR_POWER: return "2: MOTOR PWR";
    case TEST_POWER_LED: return "3: PWR LED";
    case TEST_FIRMWARE_INSTALL: return "4: FIRMWARE";
    case TEST_DEBUG_LED: return "5: DBG LED";
    case TEST_SELF_CALIBRATION: return "6: SELF CAL";
    case TEST_DIAGNOSTICS: return "7: I2C DIAG";
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

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);

  // TODO: move IO out of this method
  uint16_t pwr_led = analogRead(PIN_PHOTODIODE_PWR);
  if (pwr_led < 2500) {
    sprite.fillSmoothCircle(10, 58, 3, TFT_RED, TFT_BLACK);
  }
  sprite.setCursor(20, 50);
  sprite.print(pwr_led);

  uint16_t dbg_led = analogRead(PIN_PHOTODIODE_DBG);
  if (dbg_led < 2500) {
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
  if (versionInfo.valid) {
    sprite.setCursor(5, 90);
    sprite.setTextColor(TFT_CYAN, TFT_BLACK);
    sprite.print("Ver:");
    sprite.print(versionInfo.protocolVersion);
  }

  if (motorState.valid) {
    sprite.setCursor(5, 110);
    sprite.setTextColor(TFT_CYAN, TFT_BLACK);
    sprite.print("Up:");
    sprite.print(motorState.uptime / 1000.0, 1);
    sprite.print("s");

    sprite.setCursor(5, 130);
    sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
    sprite.print(getModeString(motorState.mode));

    sprite.setCursor(5, 150);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.print("Pos:");
    sprite.print(motorState.position);
    sprite.print(" ADC:");
    sprite.print(motorState.rawADC);

    sprite.setCursor(5, 170);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.print("Tch:");
    sprite.setTextColor(motorState.touch ? TFT_GREEN : TFT_RED, TFT_BLACK);
    sprite.print(motorState.touch ? "Y" : "N");

    // Touch diagnostics (delta, reference, recal count)
    sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    sprite.print(" d:");
    sprite.print(motorState.touchDelta);
    sprite.print(" r:");
    sprite.print(motorState.touchReference);
    sprite.print(" rc:");
    sprite.print(motorState.touchRecalCount);
  }

  render_count++;
  if (render_count & 0x01) {
    sprite.fillSmoothCircle(125, 235, 2, TFT_WHITE, TFT_BLACK);
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

  // Check ranges: 3.1-3.5V, 5-15mA
  if (avgV0 < 3.1 || avgV0 > 3.5) {
    testTracking.failedTestName = "LOG VOLT";
    Serial.println("FAILED: Logic voltage out of range");
  } else if (avgI0 < 5.0 || avgI0 > 15.0) {
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

  if (pwr_led >= 2500) {
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

bool testFirmwareInstall() {
  // Initialize on first call
  if (testTracking.firmwarePhase == TestTracking::FIRMWARE_PHASE_PING &&
      testTracking.firmwarePhaseStartTime == 0) {
    testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_PING;
    testTracking.firmwarePhaseStartTime = millis();
    testTracking.firmwareCommandSent = false;
    clearSerialBuffer();  // Clear buffer
    Serial.println("=== Starting firmware upload process ===");
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
        // Upload succeeded - move to serial number read phase
        Serial.println("PASSED: Firmware upload succeeded");
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_READ_SERIAL;
        testTracking.firmwarePhaseStartTime = millis();
      } else if (checkForSerialCommand(">>FAILURE<<")) {
        // Upload failed
        testTracking.failedTestName = "FW UPLOAD";
        Serial.println("FAILED: Firmware upload failed");
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
        return true;  // Test complete (failed)
      } else if (millis() - testTracking.firmwarePhaseStartTime > 20000) {
        // Timeout
        testTracking.failedTestName = "FW TIMEOUT";
        Serial.println("FAILED: Firmware upload timed out");
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_COMPLETE;
        return true;  // Test complete (failed)
      }
      return false;  // Still waiting

    case TestTracking::FIRMWARE_PHASE_READ_SERIAL:
      {
        // Read serial number from motor fader and report to host
        uint8_t serial[10];
        if (motorFader.readSerialNumber(serial)) {
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
  const uint16_t THRESHOLD = 2500;
  const uint16_t HYSTERESIS = 100;
  const uint16_t THRESHOLD_HIGH = THRESHOLD + HYSTERESIS;  // 2600
  const uint16_t THRESHOLD_LOW = THRESHOLD - HYSTERESIS;   // 2400
  const uint32_t TEST_DURATION_MS = 2000;
  const uint8_t MIN_TRANSITIONS = 7;
  const uint8_t MAX_TRANSITIONS = 9;

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

void readMotorFaderVersionInfo() {
  versionInfo.valid = false;

  if (motorFader.readProtocolVersion(versionInfo.protocolVersion)) {
    Serial.print("Motor Fader Protocol Version: ");
    Serial.println(versionInfo.protocolVersion);
    versionInfo.valid = true;
  } else {
    Serial.println("Failed to read protocol version");
  }
}

void readMotorFaderState() {
  motorState.valid = false;

  uint32_t state;
  if (!motorFader.readState(state)) {
    Serial.println("Failed to read motor fader state");
    return;
  }

  // Extract fields from state bitfield
  motorState.touch = (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp;
  motorState.mode = (state & STATE_MODE_bm) >> STATE_MODE_bp;
  motorState.position = (state & STATE_POSITION_bm) >> STATE_POSITION_bp;
  motorState.rawADC = (state & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp;

  // Read uptime separately
  if (!motorFader.readUptime(motorState.uptime)) {
    Serial.println("Failed to read uptime");
    return;
  }

  // Read touch diagnostics
  if (!motorFader.readTouchDelta(motorState.touchDelta)) {
    Serial.println("Failed to read touch delta");
    return;
  }

  if (!motorFader.readTouchReference(motorState.touchReference)) {
    Serial.println("Failed to read touch reference");
    return;
  }

  if (!motorFader.readTouchRecalCount(motorState.touchRecalCount)) {
    Serial.println("Failed to read touch recal count");
    return;
  }

  motorState.valid = true;
}

bool testSelfCalibration() {
  const uint32_t TOTAL_TIMEOUT_MS = 8000;
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
    Serial.println("Starting self-calibration test (8 seconds max)...");
  }

  // Send self-calibration command on first call
  if (!testTracking.selfCalRequestSent) {
    if (motorFader.selfCalibrate()) {
      Serial.println("Self-calibration command sent");
      testTracking.selfCalRequestSent = true;
    } else {
      testTracking.failedTestName = "CAL CMD";
      Serial.println("FAILED: Could not send self-calibration command");
      return true;  // Test complete (failed)
    }
  }

  // Read motor fader state continuously
  readMotorFaderState();

  if (!motorState.valid) {
    // Can't read state, check for overall timeout
    if (millis() - testTracking.selfCalStartTime > TOTAL_TIMEOUT_MS) {
      testTracking.failedTestName = "CAL NO I2C";
      Serial.println("FAILED: Cannot read motor fader state");
      return true;  // Test complete (failed)
    }
    return false;  // Keep trying
  }

  // Update min/max ADC values
  if (motorState.rawADC < testTracking.selfCalMinADC) {
    testTracking.selfCalMinADC = motorState.rawADC;
    Serial.print("Self-cal: New min ADC = ");
    Serial.println(motorState.rawADC);
  }
  if (motorState.rawADC > testTracking.selfCalMaxADC) {
    testTracking.selfCalMaxADC = motorState.rawADC;
    Serial.print("Self-cal: New max ADC = ");
    Serial.println(motorState.rawADC);
  }

  // Check if mode entered self-calibration
  if (!testTracking.selfCalEnteredMode) {
    if (motorState.mode == MODE_SELF_CALIBRATION) {
      testTracking.selfCalEnteredMode = true;
      Serial.println("Self-calibration mode entered");
    } else if (millis() - testTracking.selfCalStartTime > MODE_ENTRY_TIMEOUT_MS) {
      testTracking.failedTestName = "CAL NO ENTRY";
      Serial.print("FAILED: Did not enter calibration mode (current mode: ");
      Serial.print(getModeString(motorState.mode));
      Serial.println(")");
      return true;  // Test complete (failed)
    }
  }

  // Check for ERROR mode
  if (motorState.mode == MODE_ERROR) {
    testTracking.failedTestName = "CAL ERROR";
    Serial.println("FAILED: Motor fader entered ERROR mode during calibration");
    return true;  // Test complete (failed)
  }

  // Check for calibration completion (transition away from MODE_SELF_CALIBRATION)
  if (testTracking.selfCalEnteredMode && motorState.mode != MODE_SELF_CALIBRATION) {
    Serial.print("Self-calibration completed. Mode: ");
    Serial.println(getModeString(motorState.mode));
    Serial.print("ADC range observed: ");
    Serial.print(testTracking.selfCalMinADC);
    Serial.print(" - ");
    Serial.println(testTracking.selfCalMaxADC);

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

bool testDiagnostics() {
  // Start timing on first call
  if (testTracking.diagnosticsStartTime == 0) {
    testTracking.diagnosticsStartTime = millis();
    testTracking.diagnosticsMovementStep = 0;
    Serial.println("Starting I2C diagnostics (10 seconds)...");
    readMotorFaderVersionInfo();
  }

  uint32_t elapsed = millis() - testTracking.diagnosticsStartTime;

  // Command remote movements at specific times (once each)
  if (testTracking.diagnosticsMovementStep == 0 && elapsed >= 1000) {
    motorFader.writeTargetPosition(128);
    Serial.println("Commanding position 128");
    testTracking.diagnosticsMovementStep = 1;
  } else if (testTracking.diagnosticsMovementStep == 1 && elapsed >= 2000) {
    motorFader.writeTargetPosition(255);
    Serial.println("Commanding position 255");
    testTracking.diagnosticsMovementStep = 2;
  } else if (testTracking.diagnosticsMovementStep == 2 && elapsed >= 3000) {
    motorFader.writeTargetPosition(0);
    Serial.println("Commanding position 0");
    testTracking.diagnosticsMovementStep = 3;
  }

  // Continuously read motor state
  readMotorFaderState();

  // Check if 10 seconds have elapsed
  if (elapsed < 4000) {
    return false;  // Still running
  }

  Serial.println("PASSED: Diagnostics complete");
  return true;
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
    currentTestState = TEST_IDLE;
    clearSerialBuffer();  // Clear serial buffer
    versionInfo.valid = false;
    motorState.valid = false;
    lastPresenceState = presencePressed;
    return;
  }

  switch (currentTestState) {
    case TEST_IDLE:
      if (presenceJustPressed) {
        // Debounce delay before starting tests
        delay(50);
        Serial.println("\n=== Starting Test Sequence ===");
        // Reset test tracking
        testTracking.sampleCount = 0;
        testTracking.diagnosticsStartTime = 0;
        testTracking.failedTestName = "";
        testTracking.firmwarePhase = TestTracking::FIRMWARE_PHASE_PING;
        testTracking.firmwarePhaseStartTime = 0;
        testTracking.firmwareCommandSent = false;
        testTracking.debugLedTestStartTime = 0;
        testTracking.debugLedTransitionCount = 0;
        testTracking.debugLedState = false;
        testTracking.selfCalStartTime = 0;
        testTracking.selfCalRequestSent = false;
        testTracking.selfCalEnteredMode = false;
        testTracking.selfCalMinADC = 0xFFFF;
        testTracking.selfCalMaxADC = 0;
        clearSerialBuffer();  // Clear serial buffer
        versionInfo.valid = false;
        motorState.valid = false;
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
          currentTestState = TEST_FIRMWARE_INSTALL;
        }
      }
      break;

    case TEST_FIRMWARE_INSTALL:
      if (testFirmwareInstall()) {
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
        Serial.println("\n=== ALL TESTS PASSED ===\n");
        currentTestState = TEST_PASSED;
      }
      break;

    case TEST_PASSED:
      // Continue refreshing motor state while in passed state
      readMotorFaderState();

      if (presenceJustReleased) {
        Serial.println("Returning to idle\n");
        currentTestState = TEST_IDLE;
        versionInfo.valid = false;
        motorState.valid = false;
      }
      break;

    case TEST_FAILED:
      if (presenceJustReleased) {
        Serial.println("Test failed, returning to idle\n");
        currentTestState = TEST_IDLE;
      }
      break;
  }

  lastPresenceState = presencePressed;
}


// ============================================================================
// Main Loop
// ============================================================================

void loop() {
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

  // servo.write(pressed ? 20 : 80);

  updateDisplay(voltage0, current0, voltage1, current1);

  delay(10);
}
