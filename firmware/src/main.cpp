#include <Arduino.h>
#include <Wire.h>
#include <ptc_touch.h>
#include <megaTinyCore.h>

#include "i2c_data.h"
#include "util.h"



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


// PWM configuration
#if defined(MILLIS_USE_TIMERA0) || defined(__AVR_ATtinyxy2__)
  #error "This sketch takes over TCA0, don't use for millis here.  Pin mappings on 8-pin parts are different"
#endif

#define MOVEMENT_TIMEOUT_MILLIS (2000)
#define TOUCH_OVERRIDE_DURATION_THRESHOLD (50)
#define REMOTE_MOVEMENT_STEADY_THRESHOLD (200)
#define IDLE_DURATION_THRESHOLD (1000)

const float ALPHA = 0.05;
float input_ewma = 0;
float input_slow_ewma = 0;

// Touch state
bool touch = false;
cap_sensor_t touch_sensor;

// I2C slave base address (before A0/A1/A2 jumpers are applied)
const uint8_t I2C_BASE_ADDRESS = 0x20;


#define INPUT_CALIB_MIN (40)
#define INPUT_CALIB_MAX (1010)

int16_t target = 512;
uint8_t current_register = REG_VERSION;  // Track which register was last accessed

const int16_t WINDOW_SIZE = 8;
int16_t position_window_upper = WINDOW_SIZE;
int16_t position_window_lower = 0;
int16_t position = 0;

uint32_t state = (Mode::MODE_INPUT_IDLE << STATE_MODE_bp);

uint32_t remote_movement_start = 0;
uint32_t touch_state_change_millis = 0;
uint32_t remote_movement_steady_time = 0;
uint32_t input_last_change_millis = 0;
uint16_t remote_movement_start_position = 0;

bool pending_report_on_idle = false;

bool pending_calibrate_touch = false;
void calibrate_touch();

Mode get_mode() {
  return static_cast<Mode>((state & STATE_MODE_bm) >> STATE_MODE_bp);
}

void set_mode(Mode mode) {
  state = (state & ~STATE_MODE_bm) | (mode << STATE_MODE_bp);
}

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
  uint8_t r = current_register;
  if (r == REG_VERSION) {
      Wire.write(I2C_PROTOCOL_VERSION);
  } else if (r == REG_STATE) {
      Wire.write((state >> 24) & 0xFF);
      Wire.write((state >> 16) & 0xFF);
      Wire.write((state >> 8) & 0xFF);
      Wire.write(state & 0xFF);
  } else if (r == REG_TARGET) {
      Wire.write((target >> 8) & 0xFF);  // High byte
      Wire.write(target & 0xFF);         // Low byte
  } else if (r == REG_UPTIME) {
      uint32_t uptime = millis();
      Wire.write((uptime >> 24) & 0xFF);
      Wire.write((uptime >> 16) & 0xFF);
      Wire.write((uptime >> 8) & 0xFF);
      Wire.write(uptime & 0xFF);
  } else if (r == REG_TOUCH_RAW) {
      uint16_t touch_raw = touch_sensor.sensorData;
      Wire.write((touch_raw >> 8) & 0xFF);  // High byte
      Wire.write(touch_raw & 0xFF);         // Low byte
  }
}

// I2C receive handler - called when master sends data
void onI2cReceive(int howMany) {
  if (howMany == 0) return;
  
  // First byte is always the register address
  current_register = Wire.read();
  
  switch (current_register) {
    case REG_TARGET:
      if (howMany == 2) {  // 8-bit value
        uint16_t new_value = 0;
        new_value = Wire.read(); //(Wire.read() << 8) | Wire.read();
        Mode mode = get_mode();
        if (mode == Mode::MODE_INPUT_IDLE) {
          target = BOUNDED_LERP_UINT16(new_value, 0, 255, INPUT_CALIB_MIN, INPUT_CALIB_MAX);
          set_mode(Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS);
          remote_movement_start = millis();
          remote_movement_start_position = input_ewma;
        } else if (mode == Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
          target = BOUNDED_LERP_UINT16(new_value, 0, 255, INPUT_CALIB_MIN, INPUT_CALIB_MAX);
          // extend the timeout
          remote_movement_start = millis();
          remote_movement_start_position = input_ewma;
        } else if (mode == Mode::MODE_INPUT_ACTIVE) {
          // Ignore commanded target when in active input mode
          pending_report_on_idle = true;
        }
      }
      break;
    case REG_CAL_TOUCH:
      pending_calibrate_touch = true;
      break;
    case REG_CLEAR_ERROR:
      if (get_mode() == MODE_ERROR) {
        set_mode(MODE_INPUT_IDLE);
      }
      break;
    case REG_VERSION:
    case REG_STATE:
    case REG_UPTIME:
    case REG_TOUCH_RAW:
      // Read-only registers, ignore writes
      // Discard any excess data
      while (Wire.available()) Wire.read();
      break;
      
    default:
      // Unknown register, discard data
      // Discard any excess data
      while (Wire.available()) Wire.read();
      break;
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

void increment_position_nonce() {
  uint8_t position_nonce = (state & STATE_POSITION_NONCE_bm) >> STATE_POSITION_NONCE_bp;
  position_nonce++;
  position_nonce &= ((1 << STATE_POSITION_NONCE_bs) - 1);
  state &= ~STATE_POSITION_NONCE_bm;
  state |= position_nonce << STATE_POSITION_NONCE_bp;
}

void motor_update() {
  uint32_t now = millis();

  input_ewma = ADC1.RES * ALPHA / 2 + input_ewma * (1-ALPHA); // Use free-running ADC1 result; (2x aggregation, so divide by 2)
  Mode mode = get_mode();
  if (input_ewma > position_window_upper) {
    position_window_upper = input_ewma;
    position_window_lower = position_window_upper - WINDOW_SIZE;
    if (mode != MODE_REMOTE_MOVEMENT_IN_PROGRESS && position != position_window_upper) {
      input_last_change_millis = now;
    }
    position = position_window_upper;
  } else if (input_ewma < position_window_lower) {
    position_window_lower = input_ewma;
    position_window_upper = position_window_lower + WINDOW_SIZE;
    if (mode != MODE_REMOTE_MOVEMENT_IN_PROGRESS && position != position_window_lower) {
      input_last_change_millis = now;
    }
    position = position_window_lower;
  }

  switch (mode) {
    case MODE_REMOTE_MOVEMENT_IN_PROGRESS:
      if (now > remote_movement_start + MOVEMENT_TIMEOUT_MILLIS) {
        set_mode(Mode::MODE_ERROR);
      } else if ((state & STATE_TOUCH_bm) && now > touch_state_change_millis + TOUCH_OVERRIDE_DURATION_THRESHOLD) {
        set_mode(Mode::MODE_INPUT_ACTIVE);
      } else {
        // int8_t dir = remote_movement_start_position > target ? -1 : 1;
        // uint16_t adjusted_target = (int32_t)remote_movement_start_position + dir * (now - remote_movement_start) * 3 / 2;
        // if (dir == 1 && adjusted_target >= target) {
        //   adjusted_target = target;
        // } else if (dir == -1 && adjusted_target <= target) {
        //   adjusted_target = target;
        // }
        float delta = (target - input_ewma) * 2;
        if (delta > 15) {
          uint8_t pwm = delta + 150 > 254 ? 254 : delta + 150;
          TCA0.SPLIT.HCMP1 = pwm; // ((now % 4) < 2) ? pwm : 0;  // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
          remote_movement_steady_time = 0;
        } else if (delta < -15) {
          uint8_t pwm = -delta + 150 > 254 ? 254 : -delta + 150;
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = pwm; // ((now % 4) < 2) ? pwm : 0;  // Motor B
          remote_movement_steady_time = 0;
        } else {
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
          if (remote_movement_steady_time == 0) {
            remote_movement_steady_time = now + REMOTE_MOVEMENT_STEADY_THRESHOLD;
          } else if (now > remote_movement_steady_time) {
            remote_movement_steady_time == 0;
            set_mode(Mode::MODE_INPUT_IDLE);
          }
        }
      }
      break;
    case MODE_INPUT_ACTIVE:
      TCA0.SPLIT.HCMP1 = 0;    // Motor A
      TCA0.SPLIT.HCMP2 = 0;    // Motor B
      if (now > input_last_change_millis + IDLE_DURATION_THRESHOLD && (state & STATE_TOUCH_bm) == 0 && now > touch_state_change_millis + IDLE_DURATION_THRESHOLD) {
        if (pending_report_on_idle) {
          pending_report_on_idle = false;
          increment_position_nonce();
        }
        set_mode(Mode::MODE_INPUT_IDLE);
      }
      // TODO: haptics
      // TODO: switch to INPUT_IDLE if idle
      break;
    case MODE_INPUT_IDLE:
      TCA0.SPLIT.HCMP1 = 0;    // Motor A
      TCA0.SPLIT.HCMP2 = 0;    // Motor B
      if (now < input_last_change_millis + IDLE_DURATION_THRESHOLD || (state & STATE_TOUCH_bm) != 0 && now > touch_state_change_millis + TOUCH_OVERRIDE_DURATION_THRESHOLD) {
        set_mode(Mode::MODE_INPUT_ACTIVE);
      }
      break;
    case MODE_ERROR:
      TCA0.SPLIT.HCMP1 = 0;    // Motor A
      TCA0.SPLIT.HCMP2 = 0;    // Motor B
      break;
    }

  // TODO: lerp bounds...
  uint8_t pos = BOUNDED_LERP_UINT16(position, INPUT_CALIB_MIN, INPUT_CALIB_MAX, 0, 255);
  state &= ~STATE_POSITION_bm;
  state |= pos << STATE_POSITION_bp;


#if SERIAL_ENABLED
  static uint32_t last_print;
  if (now - last_print > 200) {
    last_print = now;
    Serial.println("");
    Serial.println(delta, DEC);
    Serial.println(get_position(), DEC);
  }
#endif
}

void setup_touch() {
  ptc_add_selfcap_node(&touch_sensor, PIN_TO_PTC(PIN_TOUCH), 0);
  ptc_node_set_thresholds(&touch_sensor, 30, 25);
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

  setup_touch();
}

void loop() {
  if (pending_calibrate_touch) {
    pending_calibrate_touch = false;
    TCA0.SPLIT.HCMP1 = 0;    // Motor A
    TCA0.SPLIT.HCMP2 = 0;    // Motor B
    delay(10);
    ptc_node_request_recal(&touch_sensor);
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(PIN_LED, HIGH);
      delay(100);
      digitalWrite(PIN_LED, LOW);
      delay(400);
    }
  }
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
  digitalWrite(PIN_LED, (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp);
  // digitalWrite(PIN_LED, millis()%512 < 128 || touch);

}

// callback that is called by ptc_process at different points to ease user interaction
void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t* node) {
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    // MySerial.print("node touched:");
    // MySerial.println(ptc_get_node_id(node));

    // touch = true;
    touch_state_change_millis = millis();
    state |= STATE_TOUCH_bm;

  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    // MySerial.print("node released:");
    // MySerial.println(ptc_get_node_id(node));

    // touch = false;
    touch_state_change_millis = millis();
    state &= ~STATE_TOUCH_bm;
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
