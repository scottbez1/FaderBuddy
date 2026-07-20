# Debugging the I2C Bootloader — Progress Log

Working notes from bringing up the ATtiny1616 I2C bootloader on real hardware
(see [ABOUT_I2C_BOOTLOADER.md](ABOUT_I2C_BOOTLOADER.md) for the design). This
captures what's been validated, the bugs found, the experiments run, and the
current open issue with ideas for the next debugging session.

## Current status (one line)

**The full I2C update cycle passes end-to-end on hardware** (`=== PASS (7046 ms)
===`): enter → erase → stream 180 pages → whole-image CRC verify → RUN_APP → app
boots with the expected `REG_FW_VERSION`. UPDI read of `0x800` matches the image
byte-for-byte. Two real bugs were found and fixed (see below); the earlier
"bulk-erase NVM erratum" (old Bug #1) turned out to be a **misdiagnosis** — the
real cause of writes never landing was the same TWI-slave defect (missing
`PIEN`) documented in Bug #3.

## The two fixes (both in `firmware/src/bootloader/bootloader.c`)

1. **Missing `PIEN` in the TWI slave init** — `SCTRLA` set only `TWI_ENABLE_bm`.
   Per datasheet §26.5.9 (APIEN note 2 / PIEN), **`PIEN` must be set for `APIF`
   to be raised on a Stop condition** — this gates the *flag*, not just the
   interrupt. Without it the polled loop never sees a STOP, so every
   master-*write* command (`SET_PAGE_ADDR`, `SEND_FRAME`, `ERASE_APP` — all
   processed at STOP) was silently dropped while still being byte-ACKed by
   hardware. Reads worked because they're processed at the repeated-START
   address match. Fix: `SCTRLA = TWI_PIEN_bm | TWI_ENABLE_bm`.
2. **`twi_service()` wedge on `COLL`** — a top-level `if (BUSERR|COLL)` branch
   cleared those flags via `SSTATUS` and returned **without issuing an `SCMD`**.
   But `APIF`/`DIF` can only be cleared by `SDATA`/`SCMD` (never by writing
   `SSTATUS`), and `COLL` co-occurs with `APIF`/`DIF` at the end of a slave
   transmit — so that path left `CLKHOLD` asserted (SCL held low) and hung the
   bus. Fix: mirror megaTinyCore's proven handler — fold `COLL` into the
   master-read completion check and always end by writing one `SCMD`.

## Hardware test rig

- **Fader under test**: ATtiny1616, I2C address `0x20` (no address jumpers).
- **UPDI**: USB-serial adapter on `/dev/ttyUSB2`
  (`usb-1a86_USB_Serial-if00-port0`). Used for flashing + **flash/RAM readback**.
- **Jig**: ESP32 (LilyGo T-Display) on `/dev/ttyUSB0` (CP2104). Talks I2C to the
  fader on its second bus (SDA=26, SCL=27), 100 kHz. Runs the `bootloader_test`
  env (`production_tools/programAndTest`), which streams the embedded offset-app
  image over the bootloader and verifies it. Presence switch starts a run;
  progress + results print on serial @115200.

### Handy commands

```bash
source ~/.platformio/penv/bin/activate

# Flash the bootloader (chip-erase + fuses BOOTEND=0x08, APPEND=0x00)
pio run -e fb_bootloader_only -t upload

# Flash the bootloader with a temporary self-test compiled in
PLATFORMIO_BUILD_FLAGS="-DBL_NVM_SELFTEST" pio run -e fb_bootloader_only -t upload

# Read flash back over UPDI (offset is a flash byte address; 0x800 = app start)
pymcuprog read -d attiny1616 -t uart -u /dev/ttyUSB2 -m flash -o 0x800 -b 64

# Read SRAM over UPDI (offset is from RAMSTART 0x3800; 0x710 -> 0x3F10)
pymcuprog read -d attiny1616 -t uart -u /dev/ttyUSB2 -m internal_sram -o 0x710 -b 8

# Read fuses (byte 7=APPEND, byte 8=BOOTEND)
pymcuprog read -d attiny1616 -t uart -u /dev/ttyUSB2 -m fuses

# Jig: build+flash the update test, then press the presence switch
cd production_tools/programAndTest && pio run -e bootloader_test -t upload
```

**Reading flash/RAM over UPDI is the single most useful debugging tool here** —
it's ground truth, independent of the I2C path.

## Confirmed working

- **Toolchain / offset build**: app links `.text`+vectors at `0x800`;
  `USING_OPTIBOOT` strips megaTinyCore's reset-flag/reset-loop `.init3` code;
  bootloader reset vector at `0x0000`, jump-to-app targets byte `0x800`.
- **Fuses**: `BOOTEND=0x08`, `APPEND=0x00` (verified: fuse row
  `00 00 02 FF 00 F6 07 00 08`).
- **Bootloader entry / marker**: reg `0x00` returns `0xB0` when resident;
  `GET_STATUS` returns `ver=1 status=1(NO_APP) last_error=0`.
- **CRC16 algorithm** is consistent across all three implementations:
  - C `bl_crc16_update` and the Python generator both give **`0x02B9`** over the
    11520-byte image; the CCITT-FALSE test vector `"123456789"` gives `0x29B1`. ✓
- **NVM page writing works** (see below): 180 consecutive `ERWP` page writes all
  land correctly (UPDI-verified: page 0=`0x00`, page 114=`0x72`, page 179=`0xB3`).
- **Per-frame streaming** works: all 180 pages (720 frames) stream and pass the
  bootloader's per-frame CRC check (`last_error` stays `0`).

## Bug #1 (MISDIAGNOSED — the real cause was Bug #3's missing `PIEN`)

> ⚠️ **This section's root-cause conclusion was wrong.** The symptom below is
> real, but the cause is *not* an NVM erratum. Without `PIEN` (Bug #3) the
> bootloader never processed *any* master-write command — including `ERASE_APP`
> and every `SEND_FRAME` — so nothing was ever erased or written, and the verify
> CRC read (`got=0xFE9D`) was just the mapped-flash stale-read hazard over an
> all-`0xFF` section. After the `PIEN` fix, writes land and the update passes
> with the current `erase_app()` (which only erases the reset-vector page). The
> investigation table below is retained because the UPDI-verified NVM behaviors
> it records (single erase + write works; direct `nvm_write_page` works) are
> still accurate — they simply weren't the cause of the streaming failure.

### Symptom
Full update ran end-to-end but failed at verify with a CRC mismatch
(`got=0xFE9D exp=0x02B9`; first-page `got=0xF154 exp=0xBE5B`), `last_error=0`.

### Investigation (all UPDI-verified ground truth)
| Experiment | Result |
|---|---|
| CRC algorithm (C vs Python vs test vector) | ✅ identical, correct |
| UPDI read of flash `0x800` after a full update | **all `0xFF`** — writes never landed |
| Fuses | ✅ `BOOTEND=0x08`, `APPEND=0x00` (0x800 is legitimately writable app section) |
| `nvm_write_page(0x800)` alone (no erase) | ✅ pattern lands |
| single `nvm_erase_page(0x800)` then write | ✅ lands |
| erase a *different* page then write `0x800` | ✅ lands |
| **`erase_app()` (224 pages) then write** | ❌ flash stays `0xFF`, **no WRERROR** |
| `erase_app()` then write, capturing `NVMCTRL.STATUS` | `00 00 04(CTRLA=PBC) 00 00 00 A0 5A` — no busy, no WRERROR; write silently no-ops |
| **180 `ERWP` writes with NO preceding erase** | ✅ all land |

Also discovered a **stale-read hazard**: right after a (failed) write, the
bootloader reading the mapped flash returned the *page-buffer* value (`0xA0`)
while UPDI showed the real flash was `0xFF`. So the bootloader can't trust an
immediate read-back of what it just wrote — only UPDI (or a re-read after other
NVM activity) is trustworthy.

### Root cause
Erasing the whole app section (~224 back-to-back standalone `PAGEERASE` ops)
leaves the NVM controller silently unable to write afterwards — the subsequent
`ERWP` commands no-op with **no** `WRERROR`. A single erase, or many `ERWP`
(erase-and-write) ops, are fine. Exact silicon mechanism unknown (looks like an
NVM quirk/erratum with that many consecutive erase commands).

### Fix (committed in `firmware/src/bootloader/bootloader.c`)
`erase_app()` now erases **only the first (reset-vector) page** to invalidate the
app. The bulk erase is unnecessary because each streamed page is written with
`ERWP`, which erases it in place. This sidesteps the quirk entirely and is faster.

> ⚠️ Side effect / follow-up: pages *beyond* the newly written image are no
> longer blanked. Fine for a full-size image; revisit if images shrink. Also the
> interrupted-update safety story now leans entirely on writing the reset-vector
> page — ideally stream page 0 **last** and/or add a CRC footer (design §10).

## Bug #2 (FIXED): TWI slave wedges the bus mid-test

> ✅ **Root cause confirmed and fixed.** Hypothesis #2's first bullet (below) was
> correct: the `BUSERR|COLL` branch cleared flags but issued no `SCMD`, leaving
> `CLKHOLD` asserted. `COLL` is set by hardware at the slave-transmit
> completion/NACK boundary (datasheet §26.5.11 COLL bit) and co-occurs with the
> `APIF`/`DIF` that the branch failed to clear — so a read occasionally ended
> with SCL held low forever, producing the persistent ESP32 `Error -1` flood.
> `twi_service()` was restructured to follow megaTinyCore's polled handler:
> compute one `action` and always write `SCTRLB` at the end; treat
> `(COLL | RXACK)` on a slave-transmit byte as "master done" → `COMPTRANS`. No
> more wedge across full 180-page (900+ transaction) runs.

### Symptom (originally observed)
```
=== I2C Bootloader Update Test ===
Image: 11520 bytes @ 0x0800, CRC16=0x02B9, FW_VERSION=1
Initial reg 0x00 = 0xB0 (bootloader resident)      <- read #1 OK
Initial bl status: ver=1 status=1 last_error=0     <- read #2 OK
[E] requestFrom(): i2cWriteReadNonStop returned Error -1   <- read #3 fails
=== FAIL: no I2C response (11 ms) ===
[E] ...Error -1  (floods continuously thereafter)
```
The first two reads succeed; the **third** repeated-start read (the one inside
`updateFirmware()`) times out, and from then on **every** transaction fails with
ESP32 `Error -1` (`i2cWriteReadNonStop`) — i.e. the bus stays wedged.

### Key context
- With the *previous* (bulk-erase) bootloader, the jig streamed **700+
  transactions** (erase poll + 180×(setpage+4 frames) + CRC) with no wedge — so
  the TWI slave is capable of sustained operation.
- The wedge appeared **after** two changes were added together:
  1. the `erase_app()` fix (erase is now ~2 ms instead of ~600 ms), and
  2. **new "initial probe" reads** in `runBootloaderUpdateTest()` (an extra
     `readVersionByte` + `getStatus` *before* `updateFirmware()`).
  So there are now 3 reads in quick succession before streaming; it wedges on #3.
- `Error -1` on ESP32 `i2cWriteReadNonStop` is typically a **timeout** (SCL/SDA
  held or no response). Once the ESP32 controller hits it, the bus often stays
  wedged until the controller is reset — consistent with the persistent flood.

### Hypotheses to test next (roughly in priority order)
1. **The extra probe reads trigger it.** Temporarily remove the initial
   `readVersionByte`/`getStatus` in `runBootloaderUpdateTest()` (or the
   `readVersionByte` at the top of `updateFirmware()`), re-run, and see whether
   the wedge moves or disappears. Cheapest way to confirm the trigger is a
   specific read *sequence* vs. just "the 3rd transaction".
2. **TWI slave state-machine edge case in `twi_service()`.** Suspects:
   - The bus-error branch (`BUSERR|COLL`) clears flags but **does not issue an
     `SCMD`** — if it ever fires it may leave `CLKHOLD` asserted (bus wedged).
     Add an explicit `TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc` / interface reset there.
   - Repeated-START handling: a write transaction immediately followed by a
     repeated-START read — verify `s_cmd_pending`/`s_txlen`/`s_txpos` can't land
     in a state where no `SCMD` is written for an event (which stretches SCL
     forever). Every APIF/DIF path must end by writing `SCTRLB`.
   - First-byte-transmit guard (`s_txpos != 0 && RXACK`) — re-check against a
     read of length 1 (the marker read) vs length 3 (status).
3. **ESP32-side bus recovery.** On a `requestFrom` failure, recover the master
   before retrying: `WireFaderBuddy.end(); WireFaderBuddy.begin(SDA,SCL,100000);`
   or clock out 9 SCL pulses. Without recovery, one timeout dooms the rest of the
   run regardless of the slave.
4. **Electrical.** Check bus pull-ups (value / who provides them on the jig +
   fader), bus capacitance, and try 50 kHz (`setClock(50000)`). The wedge is
   persistent (not random), which argues *against* pure noise, but rule it out.
5. **Resident-loop watchdog.** Add a WDT / command-timeout so a wedged/abandoned
   bootloader session self-recovers instead of hanging the shared bus (design §9
   "safety fallbacks"). Doesn't fix the root cause but improves robustness.

### Suggested instrumentation for next session
- **Snapshot `TWI0.SSTATUS`** (and `SCTRLA/SCTRLB`) into the fixed RAM diag area
  at `0x3F10` from inside `twi_service()` when something unexpected happens, then
  read it over UPDI after a wedge (`internal_sram -o 0x710`). This reveals
  whether the slave is stuck with `APIF`/`DIF`/`CLKHOLD`/`BUSERR` set.
- **Standalone TWI stress test**: a jig mode that just hammers `readVersionByte`
  in a tight loop and counts the transaction number at first failure — decouples
  the TWI robustness question from the whole update flow.
- **Scope SDA/SCL** at the moment of failure to see if the slave is holding SCL
  low (clock stretch wedge) vs. simply NAKing.

## Bug #3 (FIXED): master-write commands silently dropped — missing `PIEN`

### Symptom
After the Bug #2 fix the update streamed all 180 pages with no wedge but failed
verify with `got=0xFE9D exp=0x02B9`, `last_error=0`. UPDI read of the whole app
section showed **all `0xFF`** — nothing landed — yet `nvm_write_page` worked
perfectly when driven directly (a `-DBL_NVM_SELFTEST` hook wrote two pages that
UPDI-verified correct). So NVM was fine; the *streaming* path wasn't writing.

### How it was found (the decisive technique)
A fixed-address `.noinit` **RAM counter block** (`-DBL_DIAG`, struct at `0x3F10`)
incremented on every `process_command`, `SET_PAGE_ADDR`, `SEND_FRAME`, and
`nvm_write_page`. After a run these were read over UPDI (`internal_sram -o
0x710`). Result: **`writes=0`, `frames=0`, `set_page=0`**, but a sentinel
`get_status` counter read back nonzero — proving (a) the UPDI SRAM read does
*not* reset the counters, so the zeros are real, and (b) `process_command` ran
**only for read transactions** (`GET_STATUS`, `GET_VERSION_CRC16`, reg `0x00`),
never for writes. `proc_calls=9`, all reads.

> ⚠️ Caveat learned: `pymcuprog read -m internal_sram` is only trustworthy if the
> value survives the read. Confirm with a sentinel counter (as above) before
> trusting RAM snapshots — unlike flash, RAM reflects boot state if the tool
> resets the part.

### Root cause
`twi_slave_init()` set `TWI0.SCTRLA = TWI_ENABLE_bm` only. Datasheet §26.5.9:
`APIEN` note 2 and the `PIEN` description state **`PIEN` must be `1` for `APIF`
to be set on a Stop condition** — it gates the *flag*, not merely the interrupt.
The bootloader processes master-writes at STOP (`APIF`, `AP=0`); without `PIEN`
that event never occurs, so `SET_PAGE_ADDR`/`SEND_FRAME`/`ERASE_APP` were parsed
by nobody even though the hardware byte-ACKed them (ACK happens in the `DIF`
receive path). Reads process at the repeated-START address match, which raises
`APIF` regardless of `PIEN`, so they always worked. `DIEN`/`APIEN` gate only
interrupts (unused here — no ISR, `I` flag clear) so they're left off.

### Fix
`TWI0.SCTRLA = TWI_PIEN_bm | TWI_ENABLE_bm;`. Verified: a full run reports
`writes=180`, `frames=720`, `set_page=180`, all CRCs pass, app boots. Clock
requirement (main clock ≥ 4× SCL) is satisfied by 20 MHz vs 100 kHz.

## File / code map for this work

- `firmware/src/bootloader/bootloader.c` — the bootloader (TWI slave in
  `twi_service()`, NVM in `nvm_write_page`/`nvm_erase_page`/`erase_app`, entry in
  `main()`). The temporary `-DBL_NVM_SELFTEST` (direct NVM writes) and `-DBL_DIAG`
  (RAM counter block at `0x3F10`) hooks used above have been removed now that the
  path works; see this doc's Bug #3 for how to re-add them if needed.
- `firmware/src/shared/bootloader_protocol.h` — wire protocol, memory map,
  CRC16, entry token (`0x3F00`).
- `platformio.ini` — envs `fb_bootloader_only`, `fb_app_only`,
  `fb_app_and_bootloader`; `firmware/tools/flash_with_fuses.py` (UPDI flash+fuses).
- `production_tools/programAndTest/` — jig: `src/fader_buddy_bootloader.cpp`
  (I2C bootloader client + `updateFirmware()`), `src/main.cpp`
  (`runBootloaderUpdateTest()` under `-DBOOTLOADER_TEST_MODE`), `bootloader_test`
  env, `tools/generate_app_image.py` (embeds the offset app image).

## Failure-signature reference (CRC16-CCITT over the 11520-byte image)

Handy for classifying a bad whole-image CRC read without UPDI:

| `got=` | Meaning |
|---|---|
| `0x02B9` | correct image |
| `0x94CD` | flash all `0xFF` (write never landed) |
| `0x86B3` | flash all `0x00` |
| `0xF201` | bytes written byte-swapped |
| `0xA1E8` / `0x5DCC` | only even / odd bytes of each word written |
