#include <Arduino.h>
#include <Wire.h>
// #include "Adafruit_seesawPeripheral.h"
#include <PID_v1.h>
#include <ptc_touch.h>
#include <megaTinyCore.h>

#include "util.h"

/*
 * I2C Register Map
 * ===============
 * 
 * Addr | Register Name | Access  | Type | Description
 * -----|---------------|---------|------|------------
 * 0x00 | VERSION       | R       | u8   | Protocol version (currently 1)
 * -----|---------------|---------|------|------------
 * 0x01 | POSITION      | R       | u16  | Current fader position (0-1024)
 * -----|---------------|---------|------|------------
 * 0x02 | TARGET        | R/W     | u16  | Target fader position (0-1024)
 * -----|---------------|---------|------|------------
 * 
 * Protocol:
 * - Read:  Write register address, then read N bytes
 * - Write: Write register address + N data bytes
 * - All multi-byte values are big-endian (MSB first)
 */

#define DEMO 0

#define PIN_LED (PIN_PB2)

#define PIN_MOTOR_nSLEEP (PIN_PB3)

// Energizing pin A moves fader toward the motor end
#define PIN_MOTOR_A (PIN_PA4)
#define PIN_MOTOR_B (PIN_PA5)

// Value increases as fader approaches the motor end
#define PIN_FADER (PIN_PA6)

#define PIN_TOUCH (PIN_PC3)

#define PIN_ADDR_0 (PIN_PC2)
#define PIN_ADDR_1 (PIN_PC1)
#define PIN_ADDR_2 (PIN_PC0)

// I2C register addresses
#define REG_VERSION  0x00  // Protocol version
#define REG_POSITION 0x01  // Current position (0-1024)
#define REG_TARGET   0x02  // Target position (0-1024)

// PWM configuration
#if defined(MILLIS_USE_TIMERA0) || defined(__AVR_ATtinyxy2__)
  #error "This sketch takes over TCA0, don't use for millis here.  Pin mappings on 8-pin parts are different"
#endif

const float ALPHA = 0.05;
float input_ewma = 0;

// Touch state
bool touch = false;
cap_sensor_t touch_sensor;

// I2C slave base address (before A0/A1/A2 jumpers are applied)
const uint8_t I2C_BASE_ADDRESS = 0x20;

int16_t target = 512;
uint8_t current_register = REG_POSITION;  // Track which register was last accessed

// Configure TCA0 for high-frequency PWM so it's not audible via the motor
void setup_tca0() {
  // TakeOver TCA0 for PWM
  takeOverTCA0();

  // Enable split mode
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
  
  // Configure TCA0 for single-slope PWM
  // HCMPnEN bit: port output register for the corresponding WO[n+3] pin
  // PA4=WO4=HCMP1, PA5=WO5=HCMP2
  TCA0.SPLIT.CTRLB = (TCA_SPLIT_HCMP1EN_bm | TCA_SPLIT_HCMP2EN_bm);
  TCA0.SPLIT.HPER  = 254;
  TCA0.SPLIT.HCMP1 = 0;
  TCA0.SPLIT.HCMP2 = 0;

  TCA0.SPLIT.CTRLA = TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV4_gc;
}

// I2C request handler - called when master requests data
void onI2cRequest() {
  uint16_t value;
  
  switch (current_register) {
    case REG_VERSION:
      Wire.write(1);  // Protocol version 1
      return;  // Early return since we've already sent the byte
      
    case REG_POSITION:
      value = (uint16_t)input_ewma;
      break;
      
    case REG_TARGET:
      value = target;
      break;
      
    default:
      value = 0;
  }
  
  Wire.write((value >> 8) & 0xFF);  // High byte
  Wire.write(value & 0xFF);         // Low byte
}

// I2C receive handler - called when master sends data
void onI2cReceive(int howMany) {
  if (howMany == 0) return;
  
  // First byte is always the register address
  current_register = Wire.read();
  
  // Handle register writes of different sizes
  if (howMany > 1) {  // At least register address + 1 data byte
    uint16_t new_value = 0;
    
    // Read up to 2 bytes of data based on register
    switch (current_register) {
      case REG_TARGET:
        if (howMany == 3) {  // 16-bit value
          new_value = (Wire.read() << 8) | Wire.read();
          if (new_value <= 1024) {  // Validate target range
            target = new_value;
          }
        }
        break;
        
      case REG_VERSION:
      case REG_POSITION:
        // Read-only registers, ignore writes
        while (Wire.available()) Wire.read(); // Discard any data
        break;
        
      default:
        // Unknown register, discard data
        while (Wire.available()) Wire.read();
        break;
    }
  }
}

void setup_i2c() {
  pinMode(PIN_ADDR_0, INPUT_PULLUP);
  pinMode(PIN_ADDR_1, INPUT_PULLUP);
  pinMode(PIN_ADDR_2, INPUT_PULLUP);

  uint8_t address = I2C_BASE_ADDRESS +
    (!digitalRead(PIN_ADDR_2) << 2) +
    (!digitalRead(PIN_ADDR_1) << 1) +
    (!digitalRead(PIN_ADDR_0) << 0);

  // Initialize I2C as slave
  Wire.begin(address);
  Wire.onRequest(onI2cRequest);
  Wire.onReceive(onI2cReceive);
}

// Returns the current fader position (0-1024)
uint16_t get_position() {
  return (uint16_t)input_ewma;
}

void motor_update() {
  int16_t pid_input = ADC1.RES / 2; // Use free-running ADC1 result
  input_ewma = pid_input * ALPHA + input_ewma * (1-ALPHA);
  float delta = (target - input_ewma) * 2;

  if (delta > 15) {
    uint8_t pwm = delta + 150 > 254 ? 254 : delta + 150;
    TCA0.SPLIT.HCMP1 = pwm;  // Motor A
    TCA0.SPLIT.HCMP2 = 0;    // Motor B
  } else if (delta < -15) {
    uint8_t pwm = -delta + 150 > 254 ? 254 : -delta + 150;
    TCA0.SPLIT.HCMP1 = 0;    // Motor A
    TCA0.SPLIT.HCMP2 = pwm;  // Motor B
  } else {
    TCA0.SPLIT.HCMP1 = 0;    // Motor A
    TCA0.SPLIT.HCMP2 = 0;    // Motor B
  }

#if SERIAL_ENABLED
  static uint32_t last_print;
  if (millis() - last_print > 200) {
    last_print = millis();
    Serial.println("");
    Serial.println(delta, DEC);
    Serial.println(get_position(), DEC);
  }
#endif
}

void calibrate_touch() {
  ptc_add_selfcap_node(&touch_sensor, PIN_TO_PTC(PIN_TOUCH), 0);
  // TODO
}

void setup() {
#if SERIAL_ENABLED
  Serial.swap(1); // RX/TX on alternate pins
  Serial.begin(115200);
  Serial.println("Hello world!");
#endif

  setup_i2c();

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_FADER, INPUT);
  pinMode(PIN_MOTOR_nSLEEP, OUTPUT);
  pinMode(PIN_MOTOR_A, OUTPUT);
  pinMode(PIN_MOTOR_B, OUTPUT);

  // Set up TCA0 for high-frequency PWM
  setup_tca0();
  
  digitalWrite(PIN_MOTOR_nSLEEP, HIGH);
  digitalWrite(PIN_LED, HIGH);

  // Init ADC1 for free-running motor fader input.
  // Using ADC1 with raw setup rather than megaTinyCore's analogRead helpers since
  // we need to leave ADC0 free for the PTC touch library.
  init_ADC1();
  ADC1.MUXPOS=0x02; //reads from PA6, ADC1 channel 2
  ADC1.CTRLB = ADC_SAMPNUM_ACC2_gc; // Accumulate 4 readings
  ADC1.CTRLA=ADC_ENABLE_bm|ADC_FREERUN_bm; //start in freerun
  ADC1.COMMAND=ADC_STCONV_bm; //start first conversion!

  calibrate_touch();
}

void loop() {
  motor_update();
  ptc_process(millis());

#if DEMO
  static uint32_t last_movement;
  static uint8_t x;
  if (millis() - last_movement >= 8192) {
    x++;
    if (x >= 4) {
      x = 0;
    }
    last_movement = millis();
    switch (x) {
      case 0:
        target = 60;
        break;
      case 1:
        target = 682;
        break;
      case 2:
        target = 341;
        break;
      case 3:
        target = 1004;
        break;
    }
  }
#endif

  // digitalWrite(PIN_LED, is_moving || has_position_override || touch);
  digitalWrite(PIN_LED, touch);
  // digitalWrite(PIN_LED, millis()%512 < 128 || touch);

}

// callback that is called by ptc_process at different points to ease user interaction
void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t* node) {
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    // MySerial.print("node touched:");
    // MySerial.println(ptc_get_node_id(node));

    touch = true;
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    // MySerial.print("node released:");
    // MySerial.println(ptc_get_node_id(node));

    touch = false;
  } else if (PTC_CB_EVENT_CONV_SELF_CMPL == eventType) {
    // Do more complex things here
  } else if (PTC_CB_EVENT_CONV_CALIB & eventType) {
    // if (PTC_CB_EVENT_ERR_CALIB_LOW == eventType) {
    //   MySerial.print("Calib error, Cc too low.");
    // } else if (PTC_CB_EVENT_ERR_CALIB_HIGH == eventType) {
    //   MySerial.print("Calib error, Cc too high.");
    // } else if (PTC_CB_EVENT_ERR_CALIB_TO == eventType) {
    //   MySerial.print("Calib error, calculation timeout.");
    // } else {
    //   MySerial.print("Calib Successful.");
    // }
    // MySerial.print(" Node: ");
    // MySerial.println(ptc_get_node_id(node));
  }
}
