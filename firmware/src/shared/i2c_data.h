#pragma once

#include <stdint.h>

#define I2C_PROTOCOL_VERSION (3)


/*
 * I2C Register Map
 * ===============
 *
 * Addr | Register Name | Access  | Type | Description
 * -----|---------------|---------|------|------------
 * 0x00 | VERSION       | R       | u8   | Protocol version (currently 3)
 * -----|---------------|---------|------|------------
 * 0x01 | STATE         | R       | u32  | Current state
 * -----|---------------|---------|------|------------
 * 0x02 | TARGET        | R/W     | u8   | Target fader position (0-1024)
 * -----|---------------|---------|------|------------
 * 0x03 | UPTIME        | R       | u32  | Uptime milliseconds
 * -----|---------------|---------|------|------------
 * 0x04 | CAL_TOUCH     | W       | N/A  | Recalibrate touch sensor
 * -----|---------------|---------|------|------------
 * 0x05 | CLEAR_ERROR   | W       | N/A  | Clear error state
 * -----|---------------|---------|------|------------
 * 0x06 | TOUCH_RAW     | R       | u16  | Raw touch value
 * -----|---------------|---------|------|------------
 * 0x07 | SELF_CAL      | W       | N/A  | Initiate self calibration of motor/potentiometer
 * -----|---------------|---------|------|------------
 * 0x08 | SERIAL        | R       | u8[10] | Chip serial number (10 bytes)
 * -----|---------------|---------|------|------------
 * 0x09 | TOUCH_DELTA   | R       | i16  | Touch delta (sensorData - reference)
 * -----|---------------|---------|------|------------
 * 0x0A | TOUCH_REF     | R       | u16  | Touch reference value
 * -----|---------------|---------|------|------------
 * 0x0B | TOUCH_RECAL   | R       | u16  | Touch recalibration count
 * -----|---------------|---------|------|------------
 *
 * Protocol:
 * - Read:  Write register address, then read N bytes
 * - Write: Write register address + N data bytes
 * - All multi-byte values are big-endian (MSB first)
 */

// I2C register addresses
#define REG_VERSION 0x00  // Protocol version
#define REG_STATE   0x01  // Current
#define REG_TARGET  0x02  // Target position (0-255)
#define REG_UPTIME  0x03
#define REG_CAL_TOUCH 0x04
#define REG_CLEAR_ERROR 0x05
#define REG_TOUCH_RAW 0x06
#define REG_SELF_CAL 0x07
#define REG_SERIAL 0x08  // Chip serial number (10 bytes)
#define REG_TOUCH_DELTA 0x09  // Touch delta (signed 16-bit)
#define REG_TOUCH_REF 0x0A  // Touch reference value (unsigned 16-bit)
#define REG_TOUCH_RECAL 0x0B  // Touch recalibration count (unsigned 16-bit)

enum Mode : uint8_t {
  MODE_REMOTE_MOVEMENT_IN_PROGRESS = 0,
  MODE_INPUT_ACTIVE                = 1,
  MODE_INPUT_IDLE                  = 2,
  MODE_ERROR                       = 3,
  MODE_SELF_CALIBRATION            = 4,
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

// Settings nonce: 2 bits at position 4
#define STATE_SETTINGS_NONCE_bp      (4)
#define STATE_SETTINGS_NONCE_bs      (2)
#define STATE_SETTINGS_NONCE_bm      (((1U << STATE_SETTINGS_NONCE_bs) - 1) << STATE_SETTINGS_NONCE_bp)

// Position: 8 bits at position 6
#define STATE_POSITION_bp            (6)
#define STATE_POSITION_bs            (8)
#define STATE_POSITION_bm            (((1U << STATE_POSITION_bs) - 1) << STATE_POSITION_bp)

// Position nonce: 2 bits at position 14
#define STATE_POSITION_NONCE_bp            (14)
#define STATE_POSITION_NONCE_bs            (2)
#define STATE_POSITION_NONCE_bm            (((1U << STATE_POSITION_NONCE_bs) - 1) << STATE_POSITION_NONCE_bp)

// Raw ADC: 11 bits at position 16
#define STATE_RAW_ADC_bp            (16)
#define STATE_RAW_ADC_bs            (11)
#define STATE_RAW_ADC_bm            (((1UL << STATE_RAW_ADC_bs) - 1) << STATE_RAW_ADC_bp)

// Single tap nonce: 2 bits at position 27
#define STATE_SINGLE_TAP_NONCE_bp   (27)
#define STATE_SINGLE_TAP_NONCE_bs   (2)
#define STATE_SINGLE_TAP_NONCE_bm   (((1UL << STATE_SINGLE_TAP_NONCE_bs) - 1) << STATE_SINGLE_TAP_NONCE_bp)

// Double tap nonce: 2 bits at position 29
#define STATE_DOUBLE_TAP_NONCE_bp   (29)
#define STATE_DOUBLE_TAP_NONCE_bs   (2)
#define STATE_DOUBLE_TAP_NONCE_bm   (((1UL << STATE_DOUBLE_TAP_NONCE_bs) - 1) << STATE_DOUBLE_TAP_NONCE_bp)
