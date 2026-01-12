#pragma once

#include <stdint.h>

#define I2C_PROTOCOL_VERSION (5)  // v5: Layer management in firmware, 16-bit haptic config


/*
 * I2C Register Map (Protocol Version 5)
 * ======================================
 *
 * Addr | Register Name       | Access  | Type | Description
 * -----|---------------------|---------|------|------------
 * 0x00 | VERSION             | R       | u8   | Protocol version (currently 5)
 * -----|---------------------|---------|------|------------
 * 0x01 | STATE               | R       | u32  | Current state (includes active layer in bits 4-6)
 * -----|---------------------|---------|------|------------
 * 0x02 | (removed in v5)     |         |      | Use REG_LAYER_TARGET instead
 * -----|---------------------|---------|------|------------
 * 0x03 | UPTIME              | R       | u32  | Uptime milliseconds
 * -----|---------------------|---------|------|------------
 * 0x04 | CAL_TOUCH           | W       | N/A  | Recalibrate touch sensor
 * -----|---------------------|---------|------|------------
 * 0x05 | CLEAR_ERROR         | W       | N/A  | Clear error state
 * -----|---------------------|---------|------|------------
 * 0x06 | TOUCH_RAW           | R       | u16  | Raw touch value
 * -----|---------------------|---------|------|------------
 * 0x07 | SELF_CAL            | W       | N/A  | Initiate self calibration of motor/potentiometer
 * -----|---------------------|---------|------|------------
 * 0x08 | SERIAL              | R       | u8[10] | Chip serial number (10 bytes)
 * -----|---------------------|---------|------|------------
 * 0x09 | TOUCH_DELTA         | R       | i16  | Touch delta (sensorData - reference)
 * -----|---------------------|---------|------|------------
 * 0x0A | TOUCH_REF           | R       | u16  | Touch reference value
 * -----|---------------------|---------|------|------------
 * 0x0B | TOUCH_RECAL         | R       | u16  | Touch recalibration count
 * -----|---------------------|---------|------|------------
 * 0x0C | HAPTIC_CONFIG       | R/W     | u32  | Haptic configuration [DEPRECATED - use REG_LAYER_HAPTIC_CONFIG]
 * -----|---------------------|---------|------|------------
 * 0x0D | ACTIVE_LAYER        | R/W     | u8   | Active layer index (0-7)
 * -----|---------------------|---------|------|------------
 * 0x0E | LAYER_TARGET        | R/W     | -    | Layer restore position (layer-addressed, see below)
 * -----|---------------------|---------|------|------------
 * 0x0F | LAYER_HAPTIC_CONFIG | R/W     | -    | Layer haptic config (layer-addressed, u16)
 * -----|---------------------|---------|------|------------
 *
 * Protocol:
 * - Simple registers:
 *   - Read:  Write register address, then read N bytes
 *   - Write: Write register address + N data bytes
 * - Layer-addressed registers (0x0E, 0x0F):
 *   - Read:  Write [register, layer], then read N bytes for that layer
 *   - Write: Write [register, layer, ...data] to write to specific layer
 * - All multi-byte values are big-endian (MSB first)
 *
 * v5 Breaking Changes from v4:
 * - HAPTIC_CONFIG changed from 32-bit to 16-bit (removed nonce and target position)
 * - STATE register bits 4-6 changed from haptic_config_nonce to active_layer
 * - New layer-addressed protocol for per-layer configuration
 */

// I2C register addresses
#define REG_VERSION 0x00  // Protocol version
#define REG_STATE   0x01  // Current
// 0x02 removed in v5 - use REG_LAYER_TARGET instead
#define REG_UPTIME  0x03
#define REG_CAL_TOUCH 0x04
#define REG_CLEAR_ERROR 0x05
#define REG_TOUCH_RAW 0x06
#define REG_SELF_CAL 0x07
#define REG_SERIAL 0x08  // Chip serial number (10 bytes)
#define REG_TOUCH_DELTA 0x09  // Touch delta (signed 16-bit)
#define REG_TOUCH_REF 0x0A  // Touch reference value (unsigned 16-bit)
#define REG_TOUCH_RECAL 0x0B  // Touch recalibration count (unsigned 16-bit)
// 0x0C removed in v5 - use REG_LAYER_HAPTIC_CONFIG instead
#define REG_ACTIVE_LAYER 0x0D  // Active layer index (R/W, u8)
#define REG_LAYER_TARGET 0x0E  // Layer restore position (layer-addressed, R/W, u8)
#define REG_LAYER_HAPTIC_CONFIG 0x0F  // Layer haptic config (layer-addressed, R/W, u16)

enum Mode : uint8_t {
  MODE_REMOTE_MOVEMENT_IN_PROGRESS = 0,
  MODE_INPUT_ACTIVE                = 1,
  MODE_INPUT_IDLE                  = 2,
  MODE_ERROR                       = 3,
  MODE_SELF_CALIBRATION            = 4,
};

enum HapticMode : uint8_t {
  HAPTIC_NO_HAPTICS              = 0,
  HAPTIC_SMOOTH_WITH_MAGNET_ENDS = 1,
  HAPTIC_DETENTS                 = 2,
};


/*
 * Bitfield definitions: bit position (_bp), bit size (_bs), bit mask (_bm)
 */

// Touch: 1 bit at position 0
#define STATE_TOUCH_bp (0)
#define STATE_TOUCH_bs (1)
#define STATE_TOUCH_bm (((1U << STATE_TOUCH_bs) - 1) << STATE_TOUCH_bp)

// Mode: 3 bits at position 1
#define STATE_MODE_bp (1)
#define STATE_MODE_bs (3)
#define STATE_MODE_bm (((1U << STATE_MODE_bs) - 1) << STATE_MODE_bp)

// Active layer: 3 bits at position 4 (replaces haptic_config_nonce in v5)
#define STATE_ACTIVE_LAYER_bp      (4)
#define STATE_ACTIVE_LAYER_bs      (3)
#define STATE_ACTIVE_LAYER_bm      (((1U << STATE_ACTIVE_LAYER_bs) - 1) << STATE_ACTIVE_LAYER_bp)

// Position: 8 bits at position 7
#define STATE_POSITION_bp            (7)
#define STATE_POSITION_bs            (8)
#define STATE_POSITION_bm            (((1U << STATE_POSITION_bs) - 1) << STATE_POSITION_bp)

// Position nonce: 2 bits at position 15
#define STATE_POSITION_NONCE_bp            (15)
#define STATE_POSITION_NONCE_bs            (2)
#define STATE_POSITION_NONCE_bm            (((1UL << STATE_POSITION_NONCE_bs) - 1) << STATE_POSITION_NONCE_bp)

// Raw ADC: 11 bits at position 17
#define STATE_RAW_ADC_bp            (17)
#define STATE_RAW_ADC_bs            (11)
#define STATE_RAW_ADC_bm            (((1UL << STATE_RAW_ADC_bs) - 1) << STATE_RAW_ADC_bp)

// Double tap nonce: 2 bits at position 28
#define STATE_DOUBLE_TAP_NONCE_bp   (28)
#define STATE_DOUBLE_TAP_NONCE_bs   (2)
#define STATE_DOUBLE_TAP_NONCE_bm   (((1UL << STATE_DOUBLE_TAP_NONCE_bs) - 1) << STATE_DOUBLE_TAP_NONCE_bp)

/*
 * Haptic Configuration Bitfields (16-bit format - PROTOCOL VERSION 5+)
 * =====================================================================
 * REMOVED in v5: HAPTIC_NONCE (v4: bits 0-2)
 * REMOVED in v5: HAPTIC_TARGET_POSITION (v4: bits 13-20)
 */

// Haptic mode: 3 bits at position 0
#define HAPTIC_MODE_bp              (0)
#define HAPTIC_MODE_bs              (3)
#define HAPTIC_MODE_bm              (((1U << HAPTIC_MODE_bs) - 1) << HAPTIC_MODE_bp)

// Detent count: 4 bits at position 3
#define HAPTIC_DETENT_COUNT_bp      (3)
#define HAPTIC_DETENT_COUNT_bs      (4)
#define HAPTIC_DETENT_COUNT_bm      (((1U << HAPTIC_DETENT_COUNT_bs) - 1) << HAPTIC_DETENT_COUNT_bp)

// Detent strength: 3 bits at position 7
#define HAPTIC_DETENT_STRENGTH_bp   (7)
#define HAPTIC_DETENT_STRENGTH_bs   (3)
#define HAPTIC_DETENT_STRENGTH_bm   (((1U << HAPTIC_DETENT_STRENGTH_bs) - 1) << HAPTIC_DETENT_STRENGTH_bp)

// Reserved: 6 bits at position 10-15 (must be 0)