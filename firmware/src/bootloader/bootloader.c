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
 * FaderBuddy I2C bootloader (ATtiny1616).
 *
 * Bare-metal (no Arduino framework). Lives in the hardware boot section, so it
 * owns the reset vector and always runs first. It talks I2C directly against the
 * TWI0 slave registers in polled mode (no ISRs, no Wire library) and uses
 * NVMCTRL self-programming to flash the application section.
 *
 * See ABOUT_I2C_BOOTLOADER.md for the full design and firmware/src/shared/
 * bootloader_protocol.h for the wire protocol shared with the host.
 */

#include <avr/io.h>
#include <avr/xmega.h>      /* _PROTECTED_WRITE / _PROTECTED_WRITE_SPM */
#include <avr/interrupt.h>  /* cli() */
#include <stdbool.h>
#include <stdint.h>

#include "../shared/bootloader_protocol.h"

/* ------------------------------------------------------------------ state */

static uint8_t  s_status     = BL_STATUS_IDLE;
static uint8_t  s_last_error = BL_ERR_NONE;

/* Page programming state. */
static uint16_t s_page_addr;         /* section offset of the target page */
static bool     s_page_addr_valid;
static uint8_t  s_frame_index;       /* 0..BL_FRAMES_PER_PAGE-1 */
static uint8_t  s_page_buf[BL_PAGE_SIZE];

/* One I2C transaction's worth of received bytes (opcode + payload). */
static uint8_t  s_rx[24];
static uint8_t  s_rxlen;
static bool     s_cmd_pending;       /* bytes received, not yet processed */

/* Response bytes for the current/next master-read. */
static uint8_t  s_tx[16];
static uint8_t  s_txlen;
static uint8_t  s_txpos;

static bool     s_run_app;           /* RUN_APP requested */

/* ------------------------------------------------------------------ NVM */

static inline void nvm_wait(void) {
  while (NVMCTRL.STATUS & (NVMCTRL_FBUSY_bm | NVMCTRL_EEBUSY_bm)) {
    /* spin. After a power-on reset this also waits out the NVM write lockout. */
  }
}

/* Erase+write one 64-byte page. `off` is a flash section offset; the page
 * buffer is filled through the data-space mapped flash window at 0x8000+off. */
static uint8_t nvm_write_page(uint16_t off, const uint8_t *data) {
  nvm_wait();
  _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_PAGEBUFCLR_gc);
  nvm_wait();

  volatile uint16_t *dst = (volatile uint16_t *)(BL_MAPPED_FLASH_BASE + off);
  for (uint8_t i = 0; i < BL_PAGE_SIZE / 2; i++) {
    uint16_t w = (uint16_t)data[2 * i] | ((uint16_t)data[2 * i + 1] << 8);
    dst[i] = w;  /* word store loads the temporary page buffer */
  }

  _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_PAGEERASEWRITE_gc);
  nvm_wait();  /* CPU is halted during the flash operation */
  return (NVMCTRL.STATUS & NVMCTRL_WRERROR_bm) ? BL_ERR_NVM : BL_ERR_NONE;
}

/* Erase one 64-byte page (leaves the page buffer cleared). */
static uint8_t nvm_erase_page(uint16_t off) {
  nvm_wait();
  *(volatile uint16_t *)(BL_MAPPED_FLASH_BASE + off) = 0xFFFF;  /* latch address */
  _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_PAGEERASE_gc);
  nvm_wait();
  uint8_t err = (NVMCTRL.STATUS & NVMCTRL_WRERROR_bm) ? BL_ERR_NVM : BL_ERR_NONE;
  _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_PAGEBUFCLR_gc);
  return err;
}

/* Invalidate the application ahead of an update by erasing its first page (the
 * one holding the reset vector). A full per-page erase of the entire section is
 * deliberately NOT done: erasing all ~224 pages back-to-back leaves the NVM
 * controller unable to write afterwards -- observed on hardware, the subsequent
 * page writes silently no-op with no WRERROR. It is also unnecessary, because
 * each page is streamed with an erase-and-write (ERWP) command that erases it in
 * place. Erasing the reset-vector page here makes app_is_valid() fail until the
 * image is (re)written, so an interrupted update is detected. */
static void erase_app(void) {
  uint8_t err = nvm_erase_page(BL_APP_START);
  if (err != BL_ERR_NONE) {
    s_last_error = err;
    return;
  }
  s_status = BL_STATUS_NO_APP;  /* reset vector is now blank -> no usable app */
  s_page_addr_valid = false;
}

/* CRC16 over a flash range (section offset + length), via the mapped window. */
static uint16_t flash_crc16(uint16_t addr, uint16_t len) {
  uint16_t crc = BL_CRC16_INIT;
  uint32_t end = (uint32_t)addr + len;
  if (end > BL_FLASH_SIZE) {
    end = BL_FLASH_SIZE;
  }
  const volatile uint8_t *p = (const volatile uint8_t *)(BL_MAPPED_FLASH_BASE + addr);
  for (uint32_t a = addr; a < end; a++) {
    crc = bl_crc16_update(crc, *p++);
  }
  return crc;
}

/* An application is "present" if its reset vector isn't blank flash. */
static bool app_is_valid(void) {
  uint16_t v = *(volatile uint16_t *)(BL_MAPPED_FLASH_BASE + BL_APP_START);
  return v != 0xFFFF;
}

/* --------------------------------------------------------------- commands */

static void respond_marker(void) {
  s_tx[0] = BL_VERSION_MARKER;
  s_txlen = 1;
}

static void process_command(void) {
  if (s_rxlen == 0) {
    respond_marker();
    return;
  }

  switch (s_rx[0]) {
    case 0x00:  /* REG_VERSION: "who are you" probe */
      respond_marker();
      break;

    case BL_CMD_GET_STATUS:
      s_tx[0] = BL_BOOTLOADER_VERSION;
      s_tx[1] = s_status;
      s_tx[2] = s_last_error;
      s_txlen = 3;
      break;

    case BL_CMD_SET_PAGE_ADDR:
      if (s_rxlen >= 3) {
        uint16_t addr = ((uint16_t)s_rx[1] << 8) | s_rx[2];
        s_last_error = BL_ERR_NONE;
        if (addr < BL_APP_START ||
            addr > (BL_FLASH_SIZE - BL_PAGE_SIZE) ||
            (addr % BL_PAGE_SIZE) != 0) {
          s_last_error = BL_ERR_ADDR_RANGE;
          s_page_addr_valid = false;
        } else {
          s_page_addr = addr;
          s_page_addr_valid = true;
          s_frame_index = 0;
        }
      }
      break;

    case BL_CMD_SEND_FRAME:
      /* opcode + 16 data + 2 crc == 19 bytes */
      if (s_rxlen >= 1 + BL_FRAME_DATA_LEN + 2) {
        if (!s_page_addr_valid) {
          s_last_error = BL_ERR_FRAME_SEQ;
          break;
        }
        uint16_t crc = BL_CRC16_INIT;
        for (uint8_t i = 0; i < BL_FRAME_DATA_LEN; i++) {
          crc = bl_crc16_update(crc, s_rx[1 + i]);
        }
        uint16_t rxcrc = ((uint16_t)s_rx[1 + BL_FRAME_DATA_LEN] << 8) |
                         s_rx[1 + BL_FRAME_DATA_LEN + 1];
        if (crc != rxcrc) {
          s_last_error = BL_ERR_CRC;
          break;
        }
        uint8_t off = s_frame_index * BL_FRAME_DATA_LEN;
        for (uint8_t i = 0; i < BL_FRAME_DATA_LEN; i++) {
          s_page_buf[off + i] = s_rx[1 + i];
        }
        if (++s_frame_index >= BL_FRAMES_PER_PAGE) {
          uint8_t err = nvm_write_page(s_page_addr, s_page_buf);
          if (err != BL_ERR_NONE) {
            s_last_error = err;
          }
          s_frame_index = 0;
          s_page_addr += BL_PAGE_SIZE;  /* auto-advance for streaming */
          if (s_page_addr > (BL_FLASH_SIZE - BL_PAGE_SIZE)) {
            s_page_addr_valid = false;
          }
        }
      }
      break;

    case BL_CMD_ERASE_APP:
      erase_app();
      break;

    case BL_CMD_GET_VERSION_CRC16:
      if (s_rxlen >= 5) {
        uint16_t addr = ((uint16_t)s_rx[1] << 8) | s_rx[2];
        uint16_t len  = ((uint16_t)s_rx[3] << 8) | s_rx[4];
        uint16_t crc  = flash_crc16(addr, len);
        s_tx[0] = BL_BOOTLOADER_VERSION;
        s_tx[1] = (uint8_t)(crc >> 8);
        s_tx[2] = (uint8_t)(crc & 0xFF);
        s_txlen = 3;
      }
      break;

    case BL_CMD_RUN_APP:
      s_run_app = true;
      break;

    default:
      respond_marker();
      break;
  }
}

/* ------------------------------------------------------------------ TWI0 */

static void twi_slave_init(void) {
  /* Hardware I2C address from PC2/PC1/PC0 with pull-ups (matches the app). */
  PORTC.PIN0CTRL |= PORT_PULLUPEN_bm;
  PORTC.PIN1CTRL |= PORT_PULLUPEN_bm;
  PORTC.PIN2CTRL |= PORT_PULLUPEN_bm;
  for (volatile uint16_t i = 0; i < 1000; i++) {
    /* let the pull-ups settle before sampling */
  }
  uint8_t pc = PORTC.IN;  /* jumper installed pulls the pin low -> address bit set */
  uint8_t addr = BL_I2C_BASE_ADDRESS
    + (((pc & (1 << 2)) ? 0 : 1) << 2)   /* PC2 */
    + (((pc & (1 << 1)) ? 0 : 1) << 1)   /* PC1 */
    + (((pc & (1 << 0)) ? 0 : 1) << 0);  /* PC0 */

  /* TWI0 default pins are PB0 (SCL) / PB1 (SDA). tinyAVR 1-series errata: their
   * PORTB.OUT bits must be 0. External bus pull-ups are provided by the board. */
  PORTB.OUTCLR = 0x03;

  TWI0.SADDR   = (uint8_t)(addr << 1);
  /* PIEN is REQUIRED for APIF to be set on a Stop condition (datasheet 26.5.9,
   * APIEN note 2) -- without it the polled loop never sees STOP, so master-write
   * commands (SET_PAGE_ADDR/SEND_FRAME/ERASE_APP, which are processed at STOP)
   * are silently dropped while still being byte-ACKed. No interrupts are used
   * (global I flag stays clear), so DIEN/APIEN are unnecessary; the DIF and
   * address-match APIF flags are set regardless. The main clock (20 MHz) is far
   * more than the required 4x SCL. */
  TWI0.SCTRLA  = TWI_PIEN_bm | TWI_ENABLE_bm;  /* polled slave, STOP flag on */
}

/* Handle a single TWI0 slave event (non-blocking); returns immediately if idle.
 *
 * Structured to mirror megaTinyCore's proven polled slave handler: compute one
 * SCMD "action" and write it once at the end. Crucially, APIF and DIF can ONLY
 * be cleared by writing SDATA or the SCMD field (datasheet 26.4.9) -- NOT by
 * writing SSTATUS -- and CLKHOLD (SCL stretched low) is released only when the
 * asserting flag clears. So every path that observes APIF/DIF must end by
 * writing SCTRLB, or the slave wedges the bus holding SCL low.
 *
 * COLL is therefore handled INSIDE the master-read path (it is set at the
 * transmit-completion/NACK boundary and co-occurs with DIF/APIF), never as a
 * separate early return: doing the latter clears COLL but leaves APIF/DIF set,
 * hanging the bus. COLL also auto-clears on the next Start. */
static void twi_service(void) {
  uint8_t s = TWI0.SSTATUS;
  uint8_t action = 0;  /* SCMD = NOACT */

  if (s & TWI_APIF_bm) {
    if (s & TWI_AP_bm) {  /* address match */
      if (s & TWI_DIR_bm) {
        /* master read: make sure a response is ready */
        if (s_cmd_pending) {
          process_command();
          s_cmd_pending = false;
          s_rxlen = 0;
        } else if (s_txlen == 0) {
          respond_marker();
        }
        s_txpos = 0;
        action = TWI_SCMD_RESPONSE_gc;  /* ACK address, ready to transmit */
      } else {
        /* master write: start of a new command */
        s_rxlen = 0;
        s_cmd_pending = false;
        s_txlen = 0;  /* invalidate any stale response */
        action = TWI_SCMD_RESPONSE_gc;  /* ACK address */
      }
    } else {  /* stop */
      if (s_cmd_pending) {
        process_command();
        s_cmd_pending = false;
        s_rxlen = 0;
      }
      action = TWI_SCMD_COMPTRANS_gc;
    }
  } else if (s & TWI_DIF_bm) {
    if (s & TWI_DIR_bm) {
      /* master read: slave transmit */
      if (s_txpos != 0 && (s & (TWI_RXACK_bm | TWI_COLL_bm))) {
        /* master NACKed the previous byte, or a collision ended the transfer */
        action = TWI_SCMD_COMPTRANS_gc;
      } else if (s_txpos < s_txlen) {
        TWI0.SDATA = s_tx[s_txpos++];  /* writing SDATA also clears DIF */
        action = TWI_SCMD_RESPONSE_gc;
      } else {
        /* master read past our prepared response -> end the transaction */
        action = TWI_SCMD_COMPTRANS_gc;
      }
    } else {
      /* master write: slave receive */
      uint8_t b = TWI0.SDATA;  /* reading SDATA also clears DIF */
      if (s_rxlen < sizeof(s_rx)) {
        s_rx[s_rxlen++] = b;
      }
      s_cmd_pending = true;
      action = TWI_SCMD_RESPONSE_gc;  /* ACK, receive next */
    }
  } else {
    /* No address/data event. Clear any stray sticky COLL/BUSERR (neither holds
     * CLKHOLD, so no SCMD is required) and leave the bus alone. */
    if (s & (TWI_BUSERR_bm | TWI_COLL_bm)) {
      TWI0.SSTATUS = TWI_BUSERR_bm | TWI_COLL_bm;
    }
    return;
  }

  TWI0.SCTRLB = action;
}

/* ------------------------------------------------------------------ entry */

static void jump_to_app(void) __attribute__((noreturn));
static void jump_to_app(void) {
  cli();
  /* Return the peripherals we touched to a clean state for the application. */
  TWI0.SCTRLA = 0;
  TWI0.SADDR  = 0;
  /* AVR function pointers are word addresses; the app's reset vector lives at
   * byte address BL_APP_START. Jumping there runs the app's own crt0, which
   * re-initializes the stack, .data and .bss. */
  ((void (*)(void))(BL_APP_START / 2))();
  for (;;) {}  /* unreachable */
}

int main(void) {
  /* Read the reset cause and warm-reset entry token *before* touching RAM much.
   * Then clear both so a later unrelated reset can't re-trigger the bootloader. */
  uint8_t  rstfr = RSTCTRL.RSTFR;
  uint32_t token = BL_ENTRY_TOKEN;
  RSTCTRL.RSTFR  = rstfr;  /* write 1 to clear the sticky flags */
  BL_ENTRY_TOKEN = 0;

  bool app_ok = app_is_valid();
  bool entry_requested = (rstfr & RSTCTRL_SWRF_bm) && (token == BL_ENTRY_TOKEN_MAGIC);

  if (app_ok && !entry_requested) {
    jump_to_app();  /* normal boot */
  }

  /* Stay resident as an I2C bootloader. */
  s_status = app_ok ? BL_STATUS_IDLE : BL_STATUS_NO_APP;
  s_page_addr_valid = false;

  twi_slave_init();

  for (;;) {
    twi_service();
    if (s_run_app && app_is_valid()) {
      jump_to_app();
    }
  }
}
