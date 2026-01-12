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

// Haptic parameters
#define HAPTIC_DEAD_ZONE (8)           // ADC units of dead zone around target
#define HAPTIC_BASE_PWM (150)           // Base PWM value for haptic force
#define HAPTIC_MAX_PWM (254)            // Maximum PWM value
#define HAPTIC_MAGNET_RANGE (60)        // Active range for magnetic endpoints (ADC units)
#define HAPTIC_BASE_MULTIPLIER (3.0f)   // Base force multiplier for haptics

// Tap detection timing
#define TAP_MAX_DURATION (200)            // Maximum tap press duration (ms)
#define DOUBLE_TAP_MAX_INTERVAL (200)     // Maximum time between taps (ms)
#define TAP_MAX_MOVEMENT (10)             // Maximum position change during tap (ADC units)

// Tap detection state machine
enum TapState : uint8_t {
  TAP_NONE = 0,                // No tap in progress
  TAP_FIRST_PRESSED,           // First tap touch detected
  TAP_WAITING_FOR_DOUBLE,      // Waiting to see if second tap occurs
  TAP_SECOND_PRESSED,          // Second tap touch detected
};

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

// Haptic configuration storage
uint32_t haptic_config = 0;  // Bit-packed haptic configuration register (currently active layer's config)
uint8_t last_haptic_nonce = 0;  // Track last seen nonce to detect changes [DEPRECATED in v5]

// Layer state storage (27 bytes total) - Protocol v5+
uint16_t layer_haptic_configs[8];      // 16 bytes - 16-bit haptic config per layer
uint8_t layer_restore_positions[8];    // 8 bytes - restore position per layer (0-255)
uint8_t active_layer = 0;              // 1 byte - currently active layer (0-7)
uint8_t pending_layer_change = 0xFF;   // 1 byte - deferred layer change (0xFF = none, 0-7 = layer)
uint8_t queried_layer = 0;             // 1 byte - for layer-addressed read protocol

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

// Tap detection state
TapState tap_state = TAP_NONE;
uint32_t tap_timestamp = 0;          // Timestamp of last tap-related event for duration/timeout tracking
uint16_t tap_position_start = 0;     // Raw ADC value when first touch started
uint8_t double_tap_nonce = 0;

uint32_t self_calibration_start = 0;
uint8_t self_calibration_stage = 0;
#define SELF_CALIBRATION_TIMEOUT (1500)
#define SELF_CALIBRATION_BUFFER (0.995)  // Buffer factor to prevent hitting physical limits
uint16_t self_calibration_adc_stage_0 = 0;
uint16_t self_calibration_adc_stage_1 = 0;

bool pending_report_on_idle = false;

bool pending_calibrate_touch = false;
void calibrate_touch();
void reset_tap_detection();

// Forward declarations for layer management functions (Protocol v5+)
void request_layer_change(uint8_t new_layer);
void apply_layer_change(uint8_t new_layer);
void write_layer_target(uint8_t layer, uint8_t target);
void write_layer_haptic_config(uint8_t layer, uint16_t config);

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

  // Reset tap detection when entering modes where taps shouldn't be detected
  if (mode != MODE_INPUT_IDLE && mode != MODE_INPUT_ACTIVE) {
    reset_tap_detection();
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
  } else if (r == REG_HAPTIC_CONFIG) {
      // Haptic configuration (unsigned 32-bit, bit-packed) [DEPRECATED in v5]
      Wire.write((haptic_config >> 24) & 0xFF);
      Wire.write((haptic_config >> 16) & 0xFF);
      Wire.write((haptic_config >> 8) & 0xFF);
      Wire.write(haptic_config & 0xFF);
  } else if (r == REG_ACTIVE_LAYER) {
      Wire.write(active_layer);
  } else if (r == REG_LAYER_TARGET) {
      // Return restore position for the previously queried layer
      Wire.write(layer_restore_positions[queried_layer]);
  } else if (r == REG_LAYER_HAPTIC_CONFIG) {
      // Return haptic config for the previously queried layer (16 bits, big-endian)
      uint16_t config = layer_haptic_configs[queried_layer];
      Wire.write((config >> 8) & 0xFF);  // High byte
      Wire.write(config & 0xFF);          // Low byte
  }
}

// I2C receive handler - called when master sends data
void onI2cReceive(int howMany) {
  if (howMany == 0) return;
  
  // First byte is always the register address
  current_register = Wire.read();
  
  switch (current_register) {
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
    case REG_HAPTIC_CONFIG:
      if (howMany == 5) {  // 1 byte register + 4 bytes data (u32)
        uint32_t new_config = 0;
        new_config = ((uint32_t)Wire.read() << 24) |
                     ((uint32_t)Wire.read() << 16) |
                     ((uint32_t)Wire.read() << 8) |
                     ((uint32_t)Wire.read());

        // Validate haptic configuration before applying
        HapticMode new_haptic_mode = static_cast<HapticMode>((new_config & HAPTIC_MODE_bm) >> HAPTIC_MODE_bp);

        if (new_haptic_mode == HAPTIC_DETENTS) {
          uint8_t detent_count = (new_config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp;

          // Validate detent count (must be 1-10)
          if (detent_count < 1 || detent_count > 10) {
            // Invalid config - reject by not updating config or nonce
            break;
          }
        }

        // Extract nonce from new config
        uint8_t new_nonce = (new_config & HAPTIC_NONCE_bm) >> HAPTIC_NONCE_bp;

        // Check if nonce changed
        if (new_nonce != last_haptic_nonce) {
          // Extract target position from haptic config
          uint8_t haptic_target = (new_config & HAPTIC_TARGET_POSITION_bm) >> HAPTIC_TARGET_POSITION_bp;

          // Apply target position and start movement (even if in INPUT_ACTIVE)
          target_adc = BOUNDED_LERP_UINT16(haptic_target, 0, 255, input_calib_min, input_calib_max);
          set_mode(Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS);
          remote_movement_start = millis();
          remote_movement_start_position = input_ewma;
          remote_movement_steady_start = millis();

          // Update stored nonce
          last_haptic_nonce = new_nonce;
        }

        // Always update the stored config
        haptic_config = new_config;
      }
      break;
    case REG_ACTIVE_LAYER:
      if (howMany == 2) {  // register + 1 byte layer index
        uint8_t new_layer = Wire.read() & 0x07;  // Clamp to 0-7
        request_layer_change(new_layer);
      }
      break;
    case REG_LAYER_TARGET:
      if (howMany == 2) {
        // Read setup: register + layer index
        queried_layer = Wire.read() & 0x07;
      } else if (howMany == 3) {
        // Write: register + layer + target position
        uint8_t layer = Wire.read() & 0x07;
        uint8_t target = Wire.read();
        write_layer_target(layer, target);
      }
      break;
    case REG_LAYER_HAPTIC_CONFIG:
      if (howMany == 2) {
        // Read setup: register + layer index
        queried_layer = Wire.read() & 0x07;
      } else if (howMany == 4) {
        // Write: register + layer + 2 bytes config (big-endian)
        uint8_t layer = Wire.read() & 0x07;
        uint16_t config = ((uint16_t)Wire.read() << 8) | Wire.read();
        write_layer_haptic_config(layer, config);
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

void increment_double_tap_nonce() {
  double_tap_nonce++;
  double_tap_nonce &= 0x03;  // Keep only 2 bits (0-3)
}

void reset_tap_detection() {
  tap_state = TAP_NONE;
}

// Calculate max PWM from 3-bit strength value (0-7)
// Returns maximum PWM value to use for haptic force
// strength 0 -> minimum usable PWM (~189), strength 7 -> full PWM limit (254)
uint8_t get_strength_max_pwm(uint8_t strength) {
  // Scale from 189 (minimum usable) to 254 (max) based on strength
  // strength 0: 189 max PWM, strength 7: 254 max PWM
  const uint8_t min_pwm = 189;  // Minimum usable PWM (was strength 2)
  return min_pwm + (strength * (HAPTIC_MAX_PWM - min_pwm)) / 7;
}

// Calculate the nearest detent position in ADC units
// detent_count: number of detents (1-10)
// current_position: current fader position in ADC units
// Returns: ADC value of the nearest detent
uint16_t get_nearest_detent_position(uint8_t detent_count, uint16_t current_position) {
  // Special case: single detent at midpoint
  if (detent_count == 1) {
    return input_calib_min + (input_calib_max - input_calib_min) / 2;
  }

  // For 2+ detents: evenly spaced including endpoints
  uint16_t range = input_calib_max - input_calib_min;

  // Calculate offset from minimum position
  int16_t offset = current_position - input_calib_min;

  // Calculate which detent index is nearest using rounding
  // detent_index = round(offset * (detent_count - 1) / range)
  // Using integer math: add half the divisor before dividing for rounding
  uint8_t detent_index = ((uint32_t)offset * (detent_count - 1) + range / 2) / range;

  // Clamp to valid range [0, detent_count-1]
  if (detent_index >= detent_count) {
    detent_index = detent_count - 1;
  }

  // Calculate position of that detent
  return input_calib_min + ((uint32_t)detent_index * range) / (detent_count - 1);
}

// ============================================================================
// Layer Management Functions (Protocol v5+)
// ============================================================================

// Request layer change - may be deferred based on current mode
void request_layer_change(uint8_t new_layer) {
  if (new_layer > 7) return;  // Invalid layer
  if (new_layer == active_layer) return;  // Already on this layer

  Mode mode = get_mode();
  if (mode == MODE_INPUT_IDLE || mode == MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
    apply_layer_change(new_layer);  // Apply immediately
  } else if (mode == MODE_INPUT_ACTIVE || mode == MODE_SELF_CALIBRATION) {
    pending_layer_change = new_layer;  // Defer until mode transition
  } else if (mode == MODE_ERROR) {
    return;  // Ignore - no deferral
  }
}

// Apply layer change and start movement to new layer's restore position
void apply_layer_change(uint8_t new_layer) {
  Mode mode = get_mode();

  // Save current position to outgoing layer (only if in input mode)
  if (mode == MODE_INPUT_ACTIVE) {
    uint8_t current_pos = BOUNDED_LERP_UINT16(position, input_calib_min, input_calib_max, 0, 255);
    layer_restore_positions[active_layer] = current_pos;
  }

  // Switch to new layer
  active_layer = new_layer;
  pending_layer_change = 0xFF;

  // Load new layer's haptic config
  haptic_config = layer_haptic_configs[new_layer];

  // Start movement to new layer's restore position
  target_adc = BOUNDED_LERP_UINT16(
    layer_restore_positions[new_layer], 0, 255,
    input_calib_min, input_calib_max
  );

  set_mode(MODE_REMOTE_MOVEMENT_IN_PROGRESS);
  remote_movement_start = millis();
  remote_movement_start_position = input_ewma;
  remote_movement_steady_start = millis();
}

// Write target position to a specific layer
void write_layer_target(uint8_t layer, uint8_t target) {
  if (layer > 7) return;

  if (layer == active_layer) {
    Mode mode = get_mode();
    if (mode == MODE_INPUT_IDLE) {
      // Start remote movement
      layer_restore_positions[layer] = target;
      target_adc = BOUNDED_LERP_UINT16(target, 0, 255, input_calib_min, input_calib_max);
      set_mode(MODE_REMOTE_MOVEMENT_IN_PROGRESS);
      remote_movement_start = millis();
      remote_movement_start_position = input_ewma;
      remote_movement_steady_start = millis();
    } else if (mode == MODE_INPUT_ACTIVE) {
      // Ignore - user has control
    } else if (mode == MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
      // Update target of in-progress movement
      layer_restore_positions[layer] = target;
      target_adc = BOUNDED_LERP_UINT16(target, 0, 255, input_calib_min, input_calib_max);
      remote_movement_start = millis();
      remote_movement_start_position = input_ewma;
      remote_movement_steady_start = millis();
    }
  } else {
    // Non-active layer - just update restore position
    layer_restore_positions[layer] = target;
  }
}

// Write haptic configuration to a specific layer (16-bit format)
void write_layer_haptic_config(uint8_t layer, uint16_t config) {
  if (layer > 7) return;

  // Validate haptic configuration
  HapticMode haptic_mode = static_cast<HapticMode>((config & HAPTIC_MODE_bm) >> HAPTIC_MODE_bp);
  if (haptic_mode == HAPTIC_DETENTS) {
    uint8_t detent_count = (config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp;
    if (detent_count < 1 || detent_count > 10) {
      return;  // Invalid config - reject
    }
  }

  // Store config for layer
  layer_haptic_configs[layer] = config;

  // If active layer, apply immediately (even during MODE_INPUT_ACTIVE)
  if (layer == active_layer) {
    haptic_config = config;
  }
}

void motor_update() {
  uint32_t now = millis();

  uint16_t adc_val = ADC1.RES;
  input_ewma = adc_val * ALPHA / 2 + input_ewma * (1-ALPHA); // Use free-running ADC1 result; (2x aggregation, so divide by 2)
  Mode mode = get_mode();

  // If we didn't get a second tap start in time, reset tap detection
  if (tap_state == TAP_WAITING_FOR_DOUBLE) {
    if (now - tap_timestamp > DOUBLE_TAP_MAX_INTERVAL) {
      reset_tap_detection();
    }
  }

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
        if (delta > 4) {
          uint8_t pwm = delta + 80 > 254 ? 254 : delta + 80;
          TCA0.SPLIT.HCMP1 = pwm;  // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
          remote_movement_steady_start = now;
        } else if (delta < -4) {
          uint8_t pwm = -delta + 80 > 254 ? 254 : -delta + 80;
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
      // Continuously update active layer's restore position
      {
        uint8_t current_pos = BOUNDED_LERP_UINT16(position, input_calib_min, input_calib_max, 0, 255);
        layer_restore_positions[active_layer] = current_pos;
      }

      if (now > input_last_change_millis + IDLE_DURATION_THRESHOLD && (state & STATE_TOUCH_bm) == 0 && now > touch_state_change_millis + IDLE_DURATION_THRESHOLD) {
        TCA0.SPLIT.HCMP1 = 0;    // Motor A
        TCA0.SPLIT.HCMP2 = 0;    // Motor B
        if (pending_report_on_idle) {
          pending_report_on_idle = false;
          increment_position_nonce();
        }
        set_mode(Mode::MODE_INPUT_IDLE);

        // Apply pending layer change if deferred
        if (pending_layer_change != 0xFF) {
          apply_layer_change(pending_layer_change);
          break;  // Exit switch - apply_layer_change sets mode to REMOTE_MOVEMENT
        }
      } else {
        // Haptics - extract current mode from haptic_config
        HapticMode haptic_mode = static_cast<HapticMode>((haptic_config & HAPTIC_MODE_bm) >> HAPTIC_MODE_bp);

        if (haptic_mode == HAPTIC_SMOOTH_WITH_MAGNET_ENDS) {
          // Magnetic endpoints - pull toward calibration limits when near
          uint8_t strength = (haptic_config & HAPTIC_DETENT_STRENGTH_bm) >> HAPTIC_DETENT_STRENGTH_bp;
          uint8_t max_pwm = get_strength_max_pwm(strength);

          if (input_ewma < input_calib_min + HAPTIC_MAGNET_RANGE && input_ewma > input_calib_min + HAPTIC_DEAD_ZONE) {
            float delta = (input_calib_min - input_ewma) * HAPTIC_BASE_MULTIPLIER;
            uint8_t pwm = (-delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : -delta + HAPTIC_BASE_PWM;
            TCA0.SPLIT.HCMP1 = 0;    // Motor A
            TCA0.SPLIT.HCMP2 = pwm;  // Motor B
          } else if (input_ewma > input_calib_max - HAPTIC_MAGNET_RANGE && input_ewma < input_calib_max - HAPTIC_DEAD_ZONE) {
            float delta = (input_calib_max - input_ewma) * HAPTIC_BASE_MULTIPLIER;
            uint8_t pwm = (delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : delta + HAPTIC_BASE_PWM;
            TCA0.SPLIT.HCMP1 = pwm;  // Motor A
            TCA0.SPLIT.HCMP2 = 0;    // Motor B
          } else {
            TCA0.SPLIT.HCMP1 = 0;    // Motor A
            TCA0.SPLIT.HCMP2 = 0;    // Motor B
          }
        } else if (haptic_mode == HAPTIC_DETENTS) {
          // Detent haptics - pull toward nearest detent position
          uint8_t detent_count = (haptic_config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp;
          uint8_t strength = (haptic_config & HAPTIC_DETENT_STRENGTH_bm) >> HAPTIC_DETENT_STRENGTH_bp;
          uint8_t max_pwm = get_strength_max_pwm(strength);

          // Get nearest detent position
          uint16_t nearest_detent = get_nearest_detent_position(detent_count, input_ewma);

          // Calculate displacement from detent (positive = need to move up, negative = need to move down)
          int16_t displacement = nearest_detent - input_ewma;

          // Apply dead zone
          if (abs(displacement) > HAPTIC_DEAD_ZONE) {
            // Calculate restorative force proportional to displacement
            float delta = displacement * HAPTIC_BASE_MULTIPLIER;

            if (delta > 0) {
              // Pull toward higher position (Motor A)
              uint8_t pwm = (delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : delta + HAPTIC_BASE_PWM;
              TCA0.SPLIT.HCMP1 = pwm;  // Motor A
              TCA0.SPLIT.HCMP2 = 0;    // Motor B
            } else {
              // Pull toward lower position (Motor B)
              uint8_t pwm = (-delta + HAPTIC_BASE_PWM > max_pwm) ? max_pwm : -delta + HAPTIC_BASE_PWM;
              TCA0.SPLIT.HCMP1 = 0;    // Motor A
              TCA0.SPLIT.HCMP2 = pwm;  // Motor B
            }
          } else {
            // Within dead zone, no force
            TCA0.SPLIT.HCMP1 = 0;    // Motor A
            TCA0.SPLIT.HCMP2 = 0;    // Motor B
          }
        } else {
          // No haptics for NO_HAPTICS mode
          TCA0.SPLIT.HCMP1 = 0;    // Motor A
          TCA0.SPLIT.HCMP2 = 0;    // Motor B
        }
      }
      break;
    case MODE_INPUT_IDLE:
      TCA0.SPLIT.HCMP1 = 0;    // Motor A
      TCA0.SPLIT.HCMP2 = 0;    // Motor B

      // Apply pending layer change (may be deferred from SELF_CALIBRATION)
      if (pending_layer_change != 0xFF) {
        apply_layer_change(pending_layer_change);
        break;  // Exit switch - apply_layer_change sets mode to REMOTE_MOVEMENT
      }

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

            // Check for pending layer change
            if (pending_layer_change != 0xFF) {
              apply_layer_change(pending_layer_change);
            } else {
              // Normal case: restore previous target position
              // Since we moved the position, do a remote movement to the previous target
              remote_movement_start = millis();
              remote_movement_start_position = input_ewma;
              remote_movement_steady_start = millis();
              // TODO: re-calculate target_adc using the new calibration bounds. Can't do that now since we lerp target to an ADC value upon receipt, without saving the 0-255 value
              set_mode(Mode::MODE_REMOTE_MOVEMENT_IN_PROGRESS);
            }
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

  // Pack active layer into state (bits 4-6, replacing haptic_config_nonce in v5)
  state &= ~STATE_ACTIVE_LAYER_bm;
  state |= ((uint32_t)active_layer << STATE_ACTIVE_LAYER_bp) & STATE_ACTIVE_LAYER_bm;

  // Pack double tap nonce into state (TODO: calculate on tap changes instead of every loop)
  state &= ~STATE_DOUBLE_TAP_NONCE_bm;
  state |= ((uint32_t)double_tap_nonce << STATE_DOUBLE_TAP_NONCE_bp) & STATE_DOUBLE_TAP_NONCE_bm;

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
  settings->touched_detect_nom = 2;
  settings->untouched_detect_nom = 2;
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

  // Initialize all layers with default configuration (Protocol v5+)
  for (uint8_t i = 0; i < 8; i++) {
    layer_haptic_configs[i] = 0;  // Default: HAPTIC_NO_HAPTICS (smooth mode), all bits 0
    layer_restore_positions[i] = 128;  // Default: midpoint
  }
  active_layer = 0;
  pending_layer_change = 0xFF;  // No pending change
  queried_layer = 0;

  // Load active layer's haptic config into global haptic_config
  haptic_config = layer_haptic_configs[0];

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
  pending_calibrate_touch = true;
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
  digitalWrite(PIN_LED, (TCA0.SPLIT.HCMP1 != 0 || TCA0.SPLIT.HCMP2 != 0) || (millis() % 512 < 128));
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

    // Tap detection
    if (tap_state == TAP_NONE) {
      // First tap touch detected
      tap_timestamp = millis();
      tap_position_start = ADC1.RES;  // Store raw ADC value (no EWMA latency)
      tap_state = TAP_FIRST_PRESSED;
    } else if (tap_state == TAP_WAITING_FOR_DOUBLE) {
      // Use raw ADC for immediate movement detection (no EWMA latency)
      uint16_t current_adc = ADC1.RES;
      uint16_t position_change = (current_adc > tap_position_start) ?
                                  (current_adc - tap_position_start) :
                                  (tap_position_start - current_adc);

      uint32_t now = millis();
      if (now - tap_timestamp <= DOUBLE_TAP_MAX_INTERVAL && position_change <= TAP_MAX_MOVEMENT) {
        tap_timestamp = now;
        tap_state = TAP_SECOND_PRESSED;
      } else {
        // Too slow for double-tap, reset
        reset_tap_detection();
      }
    }
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    // MySerial.print("node released:");
    // MySerial.println(ptc_get_node_id(node));

    // touch = false;
    touch_state_change_millis = millis();
    state &= ~STATE_TOUCH_bm;

    // Tap detection: validate tap on release
    if (tap_state == TAP_FIRST_PRESSED || tap_state == TAP_SECOND_PRESSED) {
      uint32_t now = millis();
      uint32_t tap_duration = now - tap_timestamp;

      // Use raw ADC for immediate movement detection (no EWMA latency)
      uint16_t current_adc = ADC1.RES;
      uint16_t position_change = (current_adc > tap_position_start) ?
                                  (current_adc - tap_position_start) :
                                  (tap_position_start - current_adc);

      // Validate tap duration and movement
      if (tap_duration <= TAP_MAX_DURATION && position_change <= TAP_MAX_MOVEMENT) {
        // Valid tap!
        if (tap_state == TAP_FIRST_PRESSED) {
          // First tap complete, wait for possible double-tap
          tap_timestamp = now;
          tap_state = TAP_WAITING_FOR_DOUBLE;
        } else if (tap_state == TAP_SECOND_PRESSED) {
          // Double-tap complete!
          increment_double_tap_nonce();
          reset_tap_detection();

          // Haptic kick for double-tap confirmation
          TCA0.SPLIT.HCMP1 = 250;  // Motor direction A
          TCA0.SPLIT.HCMP2 = 0;
          delay(6);
          TCA0.SPLIT.HCMP1 = 0;
          TCA0.SPLIT.HCMP2 = 250;  // Motor direction B
          delay(6);
          TCA0.SPLIT.HCMP1 = 0;    // Stop
          TCA0.SPLIT.HCMP2 = 0;

          // Set state to idle after double-tap completion
          set_mode(MODE_INPUT_IDLE);
        }
      } else {
        // Invalid tap (too long or too much movement)
        reset_tap_detection();
      }
    }
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
