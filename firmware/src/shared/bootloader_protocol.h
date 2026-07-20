/*
 * Copyright 2026 Scott Bezek
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * FaderBuddy I2C bootloader protocol.
 *
 * Shared between the bootloader (firmware/bootloader/), the application
 * (firmware/src/), and any I2C host that drives firmware updates (ESPHome
 * component, production jig, WebHID tool). See ABOUT_I2C_BOOTLOADER.md.
 *
 * All multi-byte fields are big-endian (MSB first), matching i2c_data.h.
 */

#pragma once

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Flash memory map (ATtiny1616: 16 KB flash, 64-byte pages, 256 pages)
 * ------------------------------------------------------------------------- */
#define BL_FLASH_SIZE          (16384U)
#define BL_PAGE_SIZE           (64U)
#define BL_MAPPED_FLASH_BASE   (0x8000U)  /* flash mapped into data space */

/*
 * Boot section size, in 256-byte units == FUSE.BOOTEND. The application is
 * linked to start at BL_APP_START (== BOOTEND * 256) and the same value is
 * written to the BOOTEND fuse. This is the single source of truth for the
 * boot/app split -- keep the app's linker offset, the fuse, and this constant
 * in lockstep (see platformio.ini and firmware/tools/merge_factory_hex.py).
 *
 * NOTE: sized generously for initial bring-up. Shrink toward the measured
 * bootloader size once the build settles (see ABOUT_I2C_BOOTLOADER.md section 2).
 */
#define BL_BOOTEND             (0x08)
#define BL_APP_START           ((uint16_t)(BL_BOOTEND) * 256U)          /* 0x0800 */
#define BL_APP_SIZE            (BL_FLASH_SIZE - BL_APP_START)
#define BL_APP_END             (BL_FLASH_SIZE - 1U)                     /* last app byte */
#define BL_APPEND              (0x00)  /* FUSE.APPEND: app section runs to end of flash */

/* ---------------------------------------------------------------------------
 * I2C addressing
 *
 * The bootloader answers on the same address as the application: base 0x20 plus
 * the 3-bit hardware address from pins PC2/PC1/PC0, so a fader keeps its bus
 * identity in bootloader mode.
 * ------------------------------------------------------------------------- */
#define BL_I2C_BASE_ADDRESS    (0x20)

/* ---------------------------------------------------------------------------
 * REG_VERSION (0x00) response
 *
 * Reading register 0x00 is the universal "who are you" probe:
 *   - a valid protocol version (>= 5) -> application is running
 *   - BL_VERSION_MARKER               -> bootloader is resident (no usable app)
 *   - NAK / no response               -> device absent or wedged
 * The marker is chosen outside any valid protocol version and != 0xFF.
 * ------------------------------------------------------------------------- */
#define BL_VERSION_MARKER      (0xB0)

/* Bootloader implementation version, reported by BL_CMD_GET_VERSION_CRC16. */
#define BL_BOOTLOADER_VERSION  (0x01)

/* ---------------------------------------------------------------------------
 * Bootloader command set (first byte of an I2C write is the opcode)
 * ------------------------------------------------------------------------- */
#define BL_CMD_SET_PAGE_ADDR     (0x01)  /* + u16 flash page addr (section offset) */
#define BL_CMD_SEND_FRAME        (0x02)  /* + 16 data bytes + u16 crc16 */
#define BL_CMD_RUN_APP           (0x03)  /* leave bootloader, start the application */
#define BL_CMD_ERASE_APP         (0x04)  /* erase the whole application section */
#define BL_CMD_GET_STATUS        (0x05)  /* read back: [version, status, last_error] */
#define BL_CMD_GET_VERSION_CRC16 (0x06)  /* + u16 addr + u16 len -> [version, crc_hi, crc_lo] */

/* SEND_FRAME geometry: 4 frames of 16 data bytes fill one 64-byte page. */
#define BL_FRAME_DATA_LEN      (16U)
#define BL_FRAMES_PER_PAGE     (BL_PAGE_SIZE / BL_FRAME_DATA_LEN)  /* 4 */

/* Status byte values returned by BL_CMD_GET_STATUS (byte 1). */
#define BL_STATUS_IDLE         (0x00)  /* resident, ready for a command */
#define BL_STATUS_NO_APP       (0x01)  /* resident because the app is invalid/blank */

/* Error codes returned by BL_CMD_GET_STATUS (byte 2); cleared by SET_PAGE_ADDR. */
#define BL_ERR_NONE            (0x00)
#define BL_ERR_CRC             (0x01)  /* SEND_FRAME payload CRC mismatch */
#define BL_ERR_ADDR_RANGE      (0x02)  /* target page outside the app section */
#define BL_ERR_FRAME_SEQ       (0x03)  /* SEND_FRAME without a preceding SET_PAGE_ADDR */
#define BL_ERR_NVM             (0x04)  /* NVMCTRL reported WRERROR */

/* ---------------------------------------------------------------------------
 * Warm-reset entry token
 *
 * To ask a running application to drop into the bootloader, the host writes
 * REG_ENTER_BOOTLOADER; the app stores BL_ENTRY_TOKEN_MAGIC at a fixed .noinit
 * RAM address, then issues a software reset. The bootloader (which owns the
 * reset vector) reads the token plus the software-reset flag to decide whether
 * to stay resident.
 *
 * The token lives at a FIXED absolute RAM address so the two independently
 * linked binaries agree on it. It is deliberately NOT at the very top of RAM:
 * the stack starts at RAMEND (0x3FFF) and grows down, so a token there would be
 * clobbered by the bootloader's own startup pushes. 0x3F00 leaves 255 bytes of
 * stack headroom (ample for the tiny bootloader) while staying well above both
 * binaries' .data/.bss (which grow up from RAMSTART 0x3800).
 * ------------------------------------------------------------------------- */
#define BL_ENTRY_TOKEN_ADDR    (0x3F00)
#define BL_ENTRY_TOKEN_MAGIC   (0xB007F00DUL)
#define BL_ENTRY_TOKEN         (*(volatile uint32_t *)(BL_ENTRY_TOKEN_ADDR))

/* ---------------------------------------------------------------------------
 * CRC16-CCITT (poly 0x1021, init 0xFFFF, no input/output reflection).
 * Used for per-frame integrity and whole-image verification.
 * ------------------------------------------------------------------------- */
static inline uint16_t bl_crc16_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (uint16_t)((crc << 1) ^ 0x1021);
    } else {
      crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

#define BL_CRC16_INIT (0xFFFF)
