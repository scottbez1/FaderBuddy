#include <Arduino.h>
#include <Wire.h>
// #include "Adafruit_seesawPeripheral.h"
#include <PID_v1.h>
#include <ptc_touch.h>

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

// I2C register addresses
#define REG_VERSION  0x00  // Protocol version
#define REG_POSITION 0x01  // Current position (0-1024)
#define REG_TARGET   0x02  // Target position (0-1024)

const float ALPHA = 0.05;
float input_ewma = 0;

// Touch state
bool touch = false;
cap_sensor_t touch_sensor;

// I2C slave address
const uint8_t I2C_ADDRESS = 0x20;

int16_t target = 512;
uint8_t current_register = REG_POSITION;  // Track which register was last accessed

// I2C request handler - called when master requests data
void requestEvent() {
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
void receiveEvent(int howMany) {
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

// Returns the current fader position (0-1024)
uint16_t get_position() {
  return (uint16_t)input_ewma;
}

void motor_update() {
  int16_t pid_input = analogRead(PIN_FADER);
  input_ewma = pid_input * ALPHA + input_ewma * (1-ALPHA);
  float delta = (target - input_ewma) * 2;

  // int16_t delta = target - pid_input;

  if (delta > 15) {
    analogWrite(PIN_MOTOR_A, delta + 130 > 255 ? 255 : delta + 130);
    analogWrite(PIN_MOTOR_B, 0);
  } else if (delta < -15) {
    analogWrite(PIN_MOTOR_A, 0);
    analogWrite(PIN_MOTOR_B, -delta + 130 > 255 ? 255 : -delta + 130);
  } else {
    analogWrite(PIN_MOTOR_A, LOW);
    analogWrite(PIN_MOTOR_B, LOW);
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
  // ptc_add_selfcap_node(&touch_sensor, PIN_TO_PTC(PIN_TOUCH), 0);
  // TODO
}

void touch_update() {
  // ptc_process(millis());
}

void setup() {
#if SERIAL_ENABLED
  Serial.swap(1); // RX/TX on alternate pins
  Serial.begin(115200);
  Serial.println("Hello world!");
#endif

  // Initialize I2C as slave
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_FADER, INPUT);
  pinMode(PIN_MOTOR_nSLEEP, OUTPUT);
  pinMode(PIN_MOTOR_A, OUTPUT);
  pinMode(PIN_MOTOR_B, OUTPUT);

  digitalWrite(PIN_MOTOR_A, 0);
  digitalWrite(PIN_MOTOR_B, 0);
  digitalWrite(PIN_MOTOR_nSLEEP, HIGH);

  digitalWrite(PIN_LED, HIGH);


  // pinMode(PIN_PB2, INPUT);
  // // Set up TCB0 for input capture (pulse width mode, inverted)
	// TCB0.CTRLB = 0 << TCB_ASYNC_bp      /* Asynchronous Enable: disabled */
	//              | 0 << TCB_CCMPEN_bp   /* Pin pid_output Enable: disabled */
	//              | 0 << TCB_CCMPINIT_bp /* Pin Initial State: disabled */
	//              | TCB_CNTMODE_PW_gc;   /* pid_input Capture Event */
	// TCB0.EVCTRL = 1 << TCB_CAPTEI_bp    /* Event pid_input Enable: enabled */
	//               | 1 << TCB_EDGE_bp    /* Event Edge: 1: pos=capture/count, neg=init */
	//               | 1 << TCB_FILTER_bp; /* pid_input Capture Noise Cancellation Filter: enabled */
	// TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc  /* CLK_PER/1 (From Prescaler) */
	//              | 1 << TCB_ENABLE_bp   /* Enable: enabled */
	//              | 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
	//              | 0 << TCB_SYNCUPD_bp; /* Synchronize Update: disabled */

  // // Connct PB2 to async event channel 1
	// EVSYS.ASYNCCH1 = EVSYS_ASYNCCH1_PORTB_PIN2_gc; /* Asynchronous Event from Pin PB2 */
  // // Connect async event channel 1 to async user 0 (for TBC0 measurement triggering)
  // EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH1_gc;

  calibrate_touch();
}

void loop() {
  motor_update();
  touch_update();

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
  digitalWrite(PIN_LED, millis()%512 < 128);

}

// bool Adafruit_seesawPeripheral_customRequestHook() {
//   uint8_t base_cmd = i2c_buffer[0];
//   uint8_t module_cmd = i2c_buffer[1];
//   if (base_cmd == SEESAW_MOTOR_FADER_BASE) {
//     if (module_cmd == SEESAW_MOTOR_FADER_STATE) {
//       Adafruit_seesawPeripheral_write16(get_position());
//       Wire.write((touch << 2) | (is_moving << 1) | (has_position_override << 0));
//     }
//   }
//   return false;
// }

// bool Adafruit_seesawPeripheral_customReceiveHook() {
//   uint8_t base_cmd = i2c_buffer[0];
//   uint8_t module_cmd = i2c_buffer[1];
//   if (base_cmd == SEESAW_MOTOR_FADER_BASE) {
//     if (module_cmd == SEESAW_MOTOR_FADER_POSITION) {
//       set_position(i2c_buffer[2] << 8 | i2c_buffer[3]);
//     }
//   }
//   return false;
// }

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
