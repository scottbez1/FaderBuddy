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
  TEST_IDLE,           // Waiting for presence switch press
  TEST_DIAGNOSTICS,    // Running I2C diagnostics
  // Future states can be added here for additional tests
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
  bool valid;
} motorState = {false, 0, 0, 0, 0, false};

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

uint8_t render_count = 0;
void updateDisplay(float v0, float c0, float v1, float c1) {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);

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

  // TODO: move IO out of this method
  uint16_t pwr_led = analogRead(PIN_PHOTODIODE_PWR);
  if (pwr_led < 2500) {
    sprite.fillSmoothCircle(10, 28, 3, TFT_RED, TFT_BLACK);
  }
  sprite.setCursor(20, 20);
  sprite.print(pwr_led);

  uint16_t dbg_led = analogRead(PIN_PHOTODIODE_DBG);
  if (dbg_led < 2500) {
    sprite.fillSmoothCircle(10, 48, 3, TFT_RED, TFT_BLACK);
  }
  sprite.setCursor(20, 40);
  sprite.print(dbg_led);

  // Display motor fader diagnostics if available
  if (versionInfo.valid) {
    sprite.setCursor(5, 70);
    sprite.setTextColor(TFT_CYAN, TFT_BLACK);
    sprite.print("Ver:");
    sprite.print(versionInfo.protocolVersion);
  }

  if (motorState.valid) {
    sprite.setCursor(5, 90);
    sprite.setTextColor(TFT_CYAN, TFT_BLACK);
    sprite.print("Up:");
    sprite.print(motorState.uptime / 1000.0, 1);
    sprite.print("s");

    sprite.setCursor(5, 110);
    sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
    sprite.print(getModeString(motorState.mode));

    sprite.setCursor(5, 130);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.print("Pos:");
    sprite.print(motorState.position);
    sprite.print(" ADC:");
    sprite.print(motorState.rawADC);

    sprite.setCursor(5, 150);
    sprite.print("Touch:");
    sprite.setTextColor(motorState.touch ? TFT_GREEN : TFT_RED, TFT_BLACK);
    sprite.print(motorState.touch ? "YES" : "NO");
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

  motorState.valid = true;
}

void handleTestStateMachine(bool presencePressed) {
  bool presenceJustPressed = presencePressed && !lastPresenceState;
  bool presenceJustReleased = !presencePressed && lastPresenceState;

  switch (currentTestState) {
    case TEST_IDLE:
      if (presenceJustPressed) {
        Serial.println("\n=== Starting Diagnostics ===");
        readMotorFaderVersionInfo();
        currentTestState = TEST_DIAGNOSTICS;
      }
      break;

    case TEST_DIAGNOSTICS:
      if (presencePressed) {
        // Continuously read state while pressed
        readMotorFaderState();
      } else if (presenceJustReleased) {
        Serial.println("=== Diagnostics Complete ===\n");
        currentTestState = TEST_IDLE;
        versionInfo.valid = false;
        motorState.valid = false;
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

  // Red LED breathing effect when not pressed
  if (!pressed) {
    float breathe = sin(millis() / 3000.0 * 2.0 * PI);
    uint16_t brightness = LERP(breathe, -1.0, 1.0, 0.02 * 1023, 0.30 * 1023);
    analogWrite(PIN_LED_RED, brightness);  // 10-bit PWM (0-1023)
    digitalWrite(PIN_LED_GREEN, LOW);
  } else {
    analogWrite(PIN_LED_RED, 0);
    digitalWrite(PIN_LED_GREEN, HIGH);
  }

  servo.write(pressed ? 20 : 80);

  // Handle test state machine
  handleTestStateMachine(pressed);

  float voltage0 = ina3221.getBusVoltage(0);
  float current0 = ina3221.getCurrentAmps(0) * 1000;
  float voltage1 = ina3221.getBusVoltage(1);
  float current1 = ina3221.getCurrentAmps(1) * 1000;

  updateDisplay(voltage0, current0, voltage1, current1);

  delay(10);
}
