#include <Arduino.h>
#include <Wire.h>
#include <ptc_touch.h>
#include <megaTinyCore.h>
#include <EEPROM.h>

#include "shared/i2c_data.h"
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

#define MOVEMENT_TIMEOUT_MILLIS (8000)
#define TOUCH_OVERRIDE_DURATION_THRESHOLD (50)
#define REMOTE_MOVEMENT_STEADY_THRESHOLD (300)
#define IDLE_DURATION_THRESHOLD (1000)

const float ALPHA = 0.05;
float input_ewma = 0;
float input_slow_ewma = 0;

// Touch state
bool touch = false;
cap_sensor_t touch_sensor;
uint16_t touch_recal_count = 0;  // Count of touch recalibrations since boot

// I2C slave base address (before A0/A1/A2 jumpers are applied)
const uint8_t I2C_BASE_ADDRESS = 0x20;

// EEPROM calibration storage
#define EEPROM_CALIBRATION_ADDR 0
#define EEPROM_CALIBRATION_MAGIC 0xCAFE  // Magic number to validate EEPROM data

struct CalibrationData {
  uint16_t magic;         // Magic number for validation
  uint16_t calib_min;     // Minimum ADC value (fader at one end)
  uint16_t calib_max;     // Maximum ADC value (fader at other end)
  uint16_t checksum;      // Simple checksum for data integrity
};

uint16_t input_calib_min = 40;
uint16_t input_calib_max = 1010;

int16_t target_adc = 512;
uint8_t current_register = REG_VERSION;  // Track which register was last accessed

const int16_t WINDOW_SIZE = 8;
int16_t position_window_upper = WINDOW_SIZE;
int16_t position_window_lower = 0;
int16_t position = 0;

uint32_t state = (Mode::MODE_INPUT_IDLE << STATE_MODE_bp);
volatile uint32_t outgoing_state = state;

uint32_t remote_movement_start = 0;
uint32_t touch_state_change_millis = 0;
uint32_t remote_movement_steady_start = 0;
uint32_t input_last_change_millis = 0;
uint16_t remote_movement_start_position = 0;

uint32_t self_calibration_start = 0;
uint8_t self_calibration_stage = 0;
#define SELF_CALIBRATION_TIMEOUT (1500)
#define SELF_CALIBRATION_BUFFER (0.995)  // Buffer factor to prevent hitting physical limits
uint16_t self_calibration_adc_stage_0 = 0;
uint16_t self_calibration_adc_stage_1 = 0;

bool pending_report_on_idle = false;

bool pending_calibrate_touch = false;
void calibrate_touch();

Mode get_mode() {
  return static_cast<Mode>((state & STATE_MODE_bm) >> STATE_MODE_bp);
}

void set_mode(Mode mode) {
  state = (state & ~STATE_MODE_bm) | (mode << STATE_MODE_bp);
  if (mode == MODE_REMOTE_MOVEMENT_IN_PROGRESS || mode == MODE_SELF_CALIBRATION) {
    TCA0.SPLIT.CTRLA = TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV256_gc;
  } else {
    TCA0.SPLIT.CTRLA = TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV4_gc;
  }
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

  TCA0.SPLIT.CTRLA = TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV256_gc;
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
      Wire.write((target_adc >> 8) & 0xFF);  // High byte
      Wire.write(target_adc & 0xFF);         // Low byte
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
  } else if (r == REG_SERIAL) {
      // Read 10-byte serial number from SIGROW
      Wire.write(SIGROW.SERNUM0);
      Wire.write(SIGROW.SERNUM1);
      Wire.write(SIGROW.SERNUM2);
      Wire.write(SIGROW.SERNUM3);
      Wire.write(SIGROW.SERNUM4);
      Wire.write(SIGROW.SERNUM5);
      Wire.write(SIGROW.SERNUM6);
      Wire.write(SIGROW.SERNUM7);
      Wire.write(SIGROW.SERNUM8);
      Wire.write(SIGROW.SERNUM9);
  } else if (r == REG_TOUCH_DELTA) {
      // Touch delta (signed 16-bit): sensorData - reference
      int16_t delta = ptc_get_node_delta(&touch_sensor);
      Wire.write((delta >> 8) & 0xFF);  // High byte
      Wire.write(delta & 0xFF);         // Low byte
  } else if (r == REG_TOUCH_REF) {
      // Touch reference value (unsigned 16-bit)
      uint16_t reference = touch_sensor.reference;
      Wire.write((reference >> 8) & 0xFF);  // High byte
      Wire.write(reference & 0xFF);         // Low byte
  } else if (r == REG_TOUCH_RECAL) {
      // Touch recalibration count (unsigned 16-bit)
      Wire.write((touch_recal_count >> 8) & 0xFF);  // High byte
      Wire.write(touch_recal_count & 0xFF);         // Low byte
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
          target_adc = BOUNDED_LERP_UINT16(new_value, 0, 255, input_calib_min, input_calib_max);
          set_mode(Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS);
          remote_movement_start = millis();
          remote_movement_start_position = input_ewma;
          remote_movement_steady_start = millis();
        } else if (mode == Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
          target_adc = BOUNDED_LERP_UINT16(new_value, 0, 255, input_calib_min, input_calib_max);
          // extend the timeout
          remote_movement_start = millis();
          remote_movement_start_position = input_ewma;
          remote_movement_steady_start = millis();
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
    case REG_SELF_CAL:
      if (get_mode() != MODE_ERROR) {
        self_calibration_stage = 0;
        self_calibration_start = millis();
        set_mode(MODE_SELF_CALIBRATION);
      }
      break;
    case REG_VERSION:
    case REG_STATE:
    case REG_UPTIME:
    case REG_TOUCH_RAW:
    case REG_SERIAL:
    case REG_TOUCH_DELTA:
    case REG_TOUCH_REF:
    case REG_TOUCH_RECAL:
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

// Save calibration data to EEPROM
void saveCalibration() {
  CalibrationData cal;
  cal.magic = EEPROM_CALIBRATION_MAGIC;
  cal.calib_min = input_calib_min;
  cal.calib_max = input_calib_max;
  // Simple checksum: sum of all data
  cal.checksum = cal.magic + cal.calib_min + cal.calib_max;

  EEPROM.put(EEPROM_CALIBRATION_ADDR, cal);
}

// Load calibration data from EEPROM
// Returns true if valid calibration was loaded, false otherwise
bool loadCalibration() {
  CalibrationData cal;
  EEPROM.get(EEPROM_CALIBRATION_ADDR, cal);

  // Validate magic number
  if (cal.magic != EEPROM_CALIBRATION_MAGIC) {
    return false;
  }

  // Validate checksum
  uint16_t expected_checksum = cal.magic + cal.calib_min + cal.calib_max;
  if (cal.checksum != expected_checksum) {
    return false;
  }

  // Validate calibration values are reasonable
  if (cal.calib_min >= cal.calib_max) {
    return false;
  }

  if (cal.calib_max - cal.calib_min < 900) {
    // Span is too small, likely invalid
    return false;
  }

  // Load calibration values
  input_calib_min = cal.calib_min;
  input_calib_max = cal.calib_max;

  return true;
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

  uint16_t adc_val = ADC1.RES;
  input_ewma = adc_val * ALPHA / 2 + input_ewma * (1-ALPHA); // Use free-running ADC1 result; (2x aggregation, so divide by 2)
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
        float delta = (target_adc - input_ewma) * 1.2;
        if (delta > 3) {
          uint8_t pwm = delta + 88 > 254 ? 254 : delta + 88;
          TCA0.SPLIT.HCMP1 = pwm;  // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
          remote_movement_steady_start = now;
        } else if (delta < -3) {
          uint8_t pwm = -delta + 88 > 254 ? 254 : -delta + 88;
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = pwm;  // Motor B
          remote_movement_steady_start = now;
        } else {
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
          if (now > remote_movement_steady_start + REMOTE_MOVEMENT_STEADY_THRESHOLD) {
            // shift hysteresis window to prevent spurious immediate "input" detection if remote movement left us near the window bounds and succeptiple to noise
            if (input_ewma < WINDOW_SIZE / 2) {
              position_window_lower = 0;
              position_window_upper = WINDOW_SIZE;
            } else if (input_ewma > 1023 - WINDOW_SIZE / 2) {
              position_window_upper = 1023;
              position_window_lower = 1023 - WINDOW_SIZE / 2;
            } else {
              position_window_lower = input_ewma - WINDOW_SIZE / 2;
              position_window_upper = position_window_lower + WINDOW_SIZE;
            }
            input_last_change_millis = now - IDLE_DURATION_THRESHOLD;
            set_mode(Mode::MODE_INPUT_IDLE);
          }
        }
      }
      break;
    case MODE_INPUT_ACTIVE:
      if (now > input_last_change_millis + IDLE_DURATION_THRESHOLD && (state & STATE_TOUCH_bm) == 0 && now > touch_state_change_millis + IDLE_DURATION_THRESHOLD) {
        TCA0.SPLIT.HCMP1 = 0;    // Motor A
        TCA0.SPLIT.HCMP2 = 0;    // Motor B
        if (pending_report_on_idle) {
          pending_report_on_idle = false;
          increment_position_nonce();
        }
        set_mode(Mode::MODE_INPUT_IDLE);
      } else {
        // Haptics
        if (input_ewma < input_calib_min + 60 && input_ewma > input_calib_min + 8) {
          float delta = (input_calib_min - input_ewma) * 2;
          uint8_t pwm = -delta + 150 > 254 ? 254 : -delta + 150;
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = pwm;  // Motor B
        } else if (input_ewma > input_calib_max - 60 && input_ewma < input_calib_max - 8) {
          float delta = (input_calib_max - input_ewma) * 3;
          uint8_t pwm = delta + 150 > 254 ? 254 : delta + 150;
          TCA0.SPLIT.HCMP1 = pwm;  // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
        } else {
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
        }
      }
      break;
    case MODE_INPUT_IDLE:
      TCA0.SPLIT.HCMP1 = 0;    // Motor A
      TCA0.SPLIT.HCMP2 = 0;    // Motor B
      if (now < input_last_change_millis + IDLE_DURATION_THRESHOLD || ((state & STATE_TOUCH_bm) != 0 && now > touch_state_change_millis + TOUCH_OVERRIDE_DURATION_THRESHOLD)) {
        set_mode(Mode::MODE_INPUT_ACTIVE);
      }
      break;
    case MODE_ERROR:
      TCA0.SPLIT.HCMP1 = 0;    // Motor A
      TCA0.SPLIT.HCMP2 = 0;    // Motor B
      break;
    case MODE_SELF_CALIBRATION:
      switch (self_calibration_stage) {
        case 0:
          if (now > self_calibration_start + SELF_CALIBRATION_TIMEOUT) {
            self_calibration_adc_stage_0 = adc_val;
            self_calibration_stage++;
            self_calibration_start = millis();
          } else {
            // Move toward lower ADC value
            TCA0.SPLIT.HCMP1 = 0;    // Motor A
            TCA0.SPLIT.HCMP2 = 254;    // Motor B
          }
          break;
        case 1:
          if (now > self_calibration_start + SELF_CALIBRATION_TIMEOUT) {
            self_calibration_adc_stage_1 = adc_val;
            self_calibration_stage++;
            self_calibration_start = millis();
          } else {
            // Move toward higher ADC value
            TCA0.SPLIT.HCMP1 = 254;    // Motor A
            TCA0.SPLIT.HCMP2 = 0;    // Motor B
          }
          break;
        case 2:
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
          if (abs((int16_t)self_calibration_adc_stage_0 - self_calibration_adc_stage_1) < 900) {
            set_mode(Mode::MODE_ERROR);
          } else {
            // Apply calibration in memory
            // Divide by 2 to account for 2-sample ADC aggregation (adc_val is 2x, but input_ewma is corrected)
            input_calib_min = (SELF_CALIBRATION_BUFFER * self_calibration_adc_stage_0 + (1.0 - SELF_CALIBRATION_BUFFER) * self_calibration_adc_stage_1) / 2;
            input_calib_max = (SELF_CALIBRATION_BUFFER * self_calibration_adc_stage_1 + (1.0 - SELF_CALIBRATION_BUFFER) * self_calibration_adc_stage_0) / 2;

            // Save calibration to EEPROM
            saveCalibration();

            // Since we moved the position, do a remote movement to the previous target
            remote_movement_start = millis();
            remote_movement_start_position = input_ewma;
            remote_movement_steady_start = millis();
            // TODO: re-calculate target_adc using the new calibration bounds. Can't do that now since we lerp target to an ADC value upon receipt, without saving the 0-255 value
            set_mode(Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS);
          }
          break;
      }
      break;
    }

  // TODO: lerp bounds...
  uint8_t pos = BOUNDED_LERP_UINT16(position, input_calib_min, input_calib_max, 0, 255);
  state &= ~STATE_POSITION_bm;
  state |= pos << STATE_POSITION_bp;

  state &= ~(uint32_t)STATE_RAW_ADC_bm;
  state |= ((uint32_t)adc_val << STATE_RAW_ADC_bp) & (uint32_t)STATE_RAW_ADC_bm;

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
  ptc_node_set_thresholds(&touch_sensor, 200, 50);
  ptc_node_set_gain(&touch_sensor, ADC_SAMPNUM_ACC1_gc, ADC_SAMPNUM_ACC1_gc);

  ptc_lib_sm_set_t* settings = ptc_get_sm_settings();
  settings->drift_down_nom = 50;
  settings->drift_up_nom = 50;
  settings->force_recal_delta = 200; // Increase drift recalibration delta?
  settings->touched_max_nom = 255; // Disable automatic recalibration on very long touch
}

void setup() {
#if SERIAL_ENABLED
  Serial.swap(1); // RX/TX on alternate pins
  Serial.begin(115200);
  Serial.println("Hello world!");
#endif

  // Load calibration from EEPROM if available
  if (loadCalibration()) {
#if SERIAL_ENABLED
    Serial.print("Loaded calibration from EEPROM: min=");
    Serial.print(input_calib_min);
    Serial.print(", max=");
    Serial.println(input_calib_max);
#endif
  } else {
#if SERIAL_ENABLED
    Serial.println("No valid calibration found, using defaults");
#endif
  }

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
  ADC1.CTRLB = ADC_SAMPNUM_ACC2_gc; // Accumulate 2 readings
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
      delay(100);
    }
  }
  motor_update();
  
  noInterrupts();
  outgoing_state = state;
  interrupts();

  ptc_process(millis());

  // digitalWrite(PIN_LED, (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp);
  digitalWrite(PIN_LED, (TCA0.SPLIT.HCMP1 != 0 || TCA0.SPLIT.HCMP2 != 0) ^ (millis() % 512 < 128));
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
    // Increment recalibration counter on successful calibration
    if (eventType == PTC_CB_EVENT_CONV_CALIB) {
      touch_recal_count++;
    }
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
