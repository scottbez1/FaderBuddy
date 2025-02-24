#include <Arduino.h>
// #include "Adafruit_seesawPeripheral.h"
#include <PID_v1.h>
#include <ptc_touch.h>

#include "util.h"

#define DEMO 1

#define PIN_LED (PIN_PB2)

#define PIN_MOTOR_nSLEEP (PIN_PB3)

// Energizing pin A moves fader toward the motor end
#define PIN_MOTOR_A (PIN_PA4)
#define PIN_MOTOR_B (PIN_PA5)

// Value increases as fader approaches the motor end
#define PIN_FADER (PIN_PA6)

#define PIN_TOUCH (PIN_PC3)


float pid_setpoint = 0;
float pid_input, pid_output;
const float pid_kp=1, pid_ki=0, pid_kd=0;
PIDT<float> pid(&pid_input, &pid_output, &pid_setpoint, pid_kp, pid_ki, pid_kd, DIRECT);

// Movement state
bool is_moving = false;
uint32_t move_start_millis = 0;
const uint32_t MOVE_TIMEOUT_MILLIS = 750;
bool has_position_override = true;
uint16_t raw_target = 0;
const uint32_t MOVE_COMPLETION_MILLIS = 100;
uint32_t at_setpoint_since_millis = 0;

const float ALPHA = 0.1;
float input_ewma = 0;

// Touch state
bool touch = false;
cap_sensor_t touch_sensor;

void set_position(uint16_t pos) {
  raw_target = pos;
  pid_setpoint = pos;

  is_moving = true;
  has_position_override = false;
  move_start_millis = millis();
}

uint16_t get_position() {
  return has_position_override ? input_ewma : raw_target;
}

void motor_update() {
  pid_input = analogRead(PIN_FADER);

  if (!is_moving) {
    // If we're not supposed to be moving, make setpoint match the input
    pid_setpoint = pid_input;
  }

  pid.Compute();

  if (pid_output > 10) {
    analogWrite(PIN_MOTOR_A, pid_output + 120);
    analogWrite(PIN_MOTOR_B, 0);
  } else if (pid_output < -10) {
    analogWrite(PIN_MOTOR_A, 0);
    analogWrite(PIN_MOTOR_B, -pid_output + 120);
  } else {
    analogWrite(PIN_MOTOR_A, LOW);
    analogWrite(PIN_MOTOR_B, LOW);
  }

  input_ewma = pid_input * ALPHA + input_ewma * (1-ALPHA);
  float delta = fabs(input_ewma - raw_target);
  if (is_moving) {
    uint32_t now = millis();
    if (now - move_start_millis > MOVE_TIMEOUT_MILLIS) {
#if SERIAL_ENABLED
      Serial.println("TIMEOUT");
#endif
      is_moving = false;
    } else {
      if (delta > 15) {
        at_setpoint_since_millis = now;
      } else if (now - at_setpoint_since_millis > MOVE_COMPLETION_MILLIS) {
        // Done moving
#if SERIAL_ENABLED
        Serial.println("Done moving");
#endif
        is_moving = false;
      }
    }
  } else if (delta > 15) {
    has_position_override = true;
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

  // Adafruit_seesawPeripheral_begin();

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_FADER, INPUT);
  pinMode(PIN_MOTOR_nSLEEP, OUTPUT);
  pinMode(PIN_MOTOR_A, OUTPUT);
  pinMode(PIN_MOTOR_B, OUTPUT);

  digitalWrite(PIN_MOTOR_A, 0);
  digitalWrite(PIN_MOTOR_B, 0);
  digitalWrite(PIN_MOTOR_nSLEEP, HIGH);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-135, 135);
  pid.SetSampleTime(10);
  pid_setpoint = 512;

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

  set_position(30);
}

void loop() {
  // Adafruit_seesawPeripheral_run();
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
        set_position(60);
        break;
      case 1:
        set_position(682);
        break;
      case 2:
        set_position(341);
        break;
      case 3:
        set_position(1004);
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
