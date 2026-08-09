# I2C Bootloader — Next Steps

Follow-up work now that the core I2C bootloader update path is working and
hardware-validated. Read these three first for context:

- [ABOUT_I2C_BOOTLOADER.md](ABOUT_I2C_BOOTLOADER.md) — the design.
- [DEBUGGING_BOOTLOADER.md](DEBUGGING_BOOTLOADER.md) — bring-up log, the two
  TWI-slave bugs that were fixed, and the debugging techniques (UPDI flash/RAM
  readback, RAM counter block). **Skim the "two fixes" section — the `PIEN` and
  `CLKHOLD` gotchas will bite you again if you touch `twi_service()`.**
- [ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md) — UPDI flashing.

## What already works (don't re-do)

- Bootloader builds/installs (`env:fb_bootloader_only`, chip-erase + BOOTEND/APPEND fuses).
- Offset app builds and runs at `0x800` (`env:fb_app_only`).
- **Full I2C update cycle, end-to-end on hardware** via the jig
  (`production_tools/programAndTest`, `env:lilygo-t-display`): enter → `ERASE_APP`
  → stream 180 pages (per-frame CRC16) → whole-image CRC verify → `RUN_APP` → app
  boots with the expected `REG_FW_VERSION`. Written flash UPDI-verified against
  the image.
- **"Enter bootloader from a running app" validated on hardware** as two steps
  of the jig's normal test sequence (`TEST_FW_BOOTSTRAP` UPDI-flashes a fixed old
  image, then `TEST_FW_I2C_UPDATE` drives `REG_ENTER_BOOTLOADER` from that
  running old app through a full update) — every board that goes through the
  jig now exercises this path. The standalone `env:bootloader_test` /
  `-DBOOTLOADER_TEST_MODE` build used for isolated bring-up has been removed
  now that `env:lilygo-t-display` exercises the same
  `FaderBuddyBootloader::updateFirmware()` path on every jig run. Details:
  `production_tools/programAndTest/src/main.cpp` (`testFwBootstrap()`,
  `testFwI2cUpdate()`), `production_tools/programAndTest/test_host.py`
  (`upload_firmware()`), `production_tools/programAndTest/factory_test_images/`.
- App-side entry registers/logic exist in `firmware/src/main.cpp`
  (`REG_ENTER_BOOTLOADER` + magic → set flag → write `.noinit` token + software
  reset; `REG_FW_VERSION` readable).

## Hardware / commands cheat-sheet

```bash
source ~/.platformio/penv/bin/activate      # required before any pio command
# UPDI adapter: /dev/ttyUSB2 (usb-1a86_USB_Serial)   ESP32 jig: /dev/ttyUSB0 (CP2104)

pio run -e fb_bootloader_only -t upload     # install bootloader (chip-erase + fuses)
pio run -e fb_app_only -t upload            # (re)flash just the app over UPDI
cd production_tools/programAndTest && pio run -e lilygo-t-display -t upload   # jig
pymcuprog read -d attiny1616 -t uart -u /dev/ttyUSB2 -m flash -o 0x800 -b 64 # ground truth
```

The jig auto-runs an update on the held-pressed presence switch ~4 s after boot
(reset the ESP32, then watch serial @115200). See DEBUGGING_BOOTLOADER.md for the
`internal_sram` RAM-counter technique (validate with a sentinel — RAM reads can
reflect boot state if the tool resets the part).

---

## P1 — ESPHome host-side firmware packaging + update flow (design §11) — DONE (untested on hardware)

Implemented per ABOUT_I2C_BOOTLOADER.md §11, config-validated and compiled
end-to-end against a real ESPHome build (`esphome compile`), but **not yet run
against real hardware** — the jig remains the only hardware-validated exerciser of
this sequence. Before relying on it:

- Flash a board with a deliberately old `FW_VERSION`, point `firmware_image:` at a
  newer `env:fb_app_only` build (via `firmware/tools/export_app_image.py`), and run
  `fader_buddy.update_firmware` for real over I2C.
- Confirm the attempt cap actually persists across an ESP32 reboot (i.e.
  `ESPPreferences` survives), and that a failure is visible via
  `on_firmware_update_result`.
- Confirm `update_firmware` refuses/defers correctly while a fader is actively being
  touched, and that it doesn't wedge the ESPHome loop watchdog on a slow/stalled
  transfer.

Notable decisions made during implementation (beyond what §11 originally sketched):

- **No autoupdate mode.** Only the manual `fader_buddy.update_firmware` action ever
  triggers an update — simpler and matches the "safest posture" reasoning in §11's
  original auto-vs-manual discussion, just taken further (no opt-in autoupdate at
  all, so no `autoupdate_firmware:` config key).
- **`FW_VERSION` is baked into the image, not a YAML field.** Added a
  `FW_VERSION_FOOTER` in `firmware/src/main.cpp`, linked to a fixed address
  (`BL_APP_META_ADDR` = last 2 bytes of flash, `bootloader_protocol.h`) via
  `-Wl,--section-start=.fw_meta=...` in `platformio.ini`. Since the app section
  always runs to `FLASHEND`, this address is stable regardless of app size, so the
  host reads the version straight from the packaged `.bin`'s last 2 bytes with zero
  chance of drifting from what the flashed app actually reports. The whole-image
  CRC16 is *not* baked in (self-referential — a CRC can't cover itself) and is
  instead computed by `__init__.py` `to_code()` directly from the `.bin` bytes.
- **New tool: `firmware/tools/export_app_image.py`** — produces the raw `.bin` that
  `firmware_image:` points to (mirrors `generate_app_image.py`'s extraction logic
  for the jig, but emits a binary instead of a C header).

## P1 — Shared-bus robustness (design §9 / §12)

These make a stuck/abandoned update non-fatal to the bus and its neighbors.

- **WDT command timeout in the resident bootloader** — if no valid command
  arrives for N seconds, self-recover (jump to app if valid, else keep serving
  but reset the TWI interface). Prevents a wedged session hanging the shared bus.
  File: `firmware/src/bootloader/bootloader.c` `main()` loop.
- **Host-side bus recovery** — on a transaction timeout, recover the master
  before retrying (`Wire.end()/begin()`, or clock 9 SCL pulses). Note: this
  cannot rescue a *slave* holding SCL — that class of wedge was the fixed
  `twi_service()` bug — but it protects against other stalls. Files: jig client
  and the future ESPHome client.

## P2 — Robust app-validity check (CRC footer, design §10)

Today `app_is_valid()` only checks the reset vector isn't blank
(`!= 0xFFFF`) — it does **not** catch a partially-written app (power lost
mid-stream). Add a linker-placed CRC footer (image length + CRC16 at end of
APPCODE) that the bootloader recomputes on boot. Pairs with `erase_app()`, which
currently erases only the reset-vector page (so pages beyond a shrunk image
aren't blanked). Consider streaming page 0 **last** so an interrupted update
leaves the reset vector blank → detected as no-app. Files:
`firmware/src/bootloader/bootloader.c` (`app_is_valid`, `erase_app`), app linker
config, `tools/generate_app_image.py`.

## P2 — Shrink `BOOTEND`

Measured bootloader size is **1258 bytes**; `BOOTEND` is `0x08` (2048).
`0x06` (1536) is a safe shrink with margin. Must change **three places in
lockstep**: `BL_BOOTEND` in `firmware/src/shared/bootloader_protocol.h`,
`board_build.text_section_start` in `platformio.ini` (the app link offset), and
the `--bootend` fuse value in the `fb_bootloader_only`/`fb_app_and_bootloader` upload
commands. Re-run a full factory flash + update to confirm. Low priority (frees
512 bytes of app space; not currently needed — app is 11.5 KB of 16 KB).

## P2 — Validate the combined factory env on a blank chip

Confirm `env:fb_app_and_bootloader` merges bootloader + offset app and writes both
**plus fuses** in a single UPDI upload onto a truly blank ATtiny1616, yielding a
running app. (Individual pieces are validated; the one-shot combined path for a
factory-fresh chip should be exercised.) File: `platformio.ini`,
`firmware/tools/flash_with_fuses.py`.

## P2 — Multi-fader, one-at-a-time

Confirm a chain of addressed faders can each be updated without disturbing the
others, and that updates are serialized on the shared bus.

## Nice-to-have — Standalone TWI stress test

A jig mode that hammers reads/writes in a tight loop and counts transactions to
first failure. Decouples TWI-robustness confidence from the whole update flow;
handy as a regression guard if `twi_service()` is ever touched.

---

## Reminders

- **`firmware/src/shared/i2c_data.h` is hand-synced across four copies** (firmware,
  `esphome/components/fader_buddy/`, the jig, the WebHID JS). Any protocol change
  must update all four. `bootloader_protocol.h` is likewise shared.
- Prefer **UPDI flash readback** as ground truth over the bootloader's own
  mapped-flash reads (stale-page-buffer hazard right after a write).
- There is no CI hardware test — everything here is validated on the bench.
