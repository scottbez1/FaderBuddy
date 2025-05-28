#pragma once

#include <stdint.h>

#define I2C_PROTOCOL_VERSION (2)


/*
 * I2C Register Map
 * ===============
 * 
 * Addr | Register Name | Access  | Type | Description
 * -----|---------------|---------|------|------------
 * 0x00 | VERSION       | R       | u8   | Protocol version (currently 1)
 * -----|---------------|---------|------|------------
 * 0x01 | STATE         | R       | u32  | Current state
 * -----|---------------|---------|------|------------
 * 0x02 | TARGET        | R/W     | u16  | Target fader position (0-1024)
 * -----|---------------|---------|------|------------
 * 0x03 | UPTIME        | R       | u32  | Uptime milliseconds
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
#define REG_TARGET  0x02  // Target position (0-1024)
#define REG_UPTIME  0x03

enum Mode : uint8_t {
  MODE_REMOTE_MOVEMENT_IN_PROGRESS = 0,
  MODE_INPUT_ACTIVE                = 1,
  MODE_INPUT_IDLE                  = 2,
  MODE_ERROR                       = 3,
};


/*
 * Bitfield definitions: bit position (_bp), bit size (_bs), bit mask (_bm)
 */

// Touch: 1 bit at position 0
#define STATE_TOUCH_bp (0)
#define STATE_TOUCH_bs (1)
#define STATE_TOUCH_bm (((1U << STATE_TOUCH_bs) - 1) << STATE_TOUCH_bp)

// Mode: 2 bits at position 1
#define STATE_MODE_bp (1)
#define STATE_MODE_bs (2)
#define STATE_MODE_bm (((1U << STATE_MODE_bs) - 1) << STATE_MODE_bp)

// Settings nonce: 2 bits at position 3
#define STATE_SETTINGS_NONCE_bp      (3)
#define STATE_SETTINGS_NONCE_bs      (2)
#define STATE_SETTINGS_NONCE_bm      (((1U << STATE_SETTINGS_NONCE_bs) - 1) << STATE_SETTINGS_NONCE_bp)

// Position: 7 bits at position 5
#define STATE_POSITION_bp            (5)
#define STATE_POSITION_bs            (7)
#define STATE_POSITION_bm            (((1U << STATE_POSITION_bs) - 1) << STATE_POSITION_bp)

// Position nonce: 2 bits at position 7
#define STATE_POSITION_NONCE_bp            (7)
#define STATE_POSITION_NONCE_bs            (2)
#define STATE_POSITION_NONCE_bm            (((1U << STATE_POSITION_NONCE_bs) - 1) << STATE_POSITION_NONCE_bp)
