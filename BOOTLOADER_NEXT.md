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
  (`production_tools/programAndTest`, `env:bootloader_test`): enter → `ERASE_APP`
  → stream 180 pages (per-frame CRC16) → whole-image CRC verify → `RUN_APP` → app
  boots with the expected `REG_FW_VERSION`. Written flash UPDI-verified against
  the image.
- App-side entry registers/logic exist in `firmware/src/main.cpp`
  (`REG_ENTER_BOOTLOADER` + magic → set flag → write `.noinit` token + software
  reset; `REG_FW_VERSION` readable).

## Hardware / commands cheat-sheet

```bash
source ~/.platformio/penv/bin/activate      # required before any pio command
# UPDI adapter: /dev/ttyUSB2 (usb-1a86_USB_Serial)   ESP32 jig: /dev/ttyUSB0 (CP2104)

pio run -e fb_bootloader_only -t upload     # install bootloader (chip-erase + fuses)
pio run -e fb_app_only -t upload            # (re)flash just the app over UPDI
cd production_tools/programAndTest && pio run -e bootloader_test -t upload   # jig
pymcuprog read -d attiny1616 -t uart -u /dev/ttyUSB2 -m flash -o 0x800 -b 64 # ground truth
```

The jig auto-runs an update on the held-pressed presence switch ~4 s after boot
(reset the ESP32, then watch serial @115200). See DEBUGGING_BOOTLOADER.md for the
`internal_sram` RAM-counter technique (validate with a sentinel — RAM reads can
reflect boot state if the tool resets the part).

---

## P0 — Validate "enter bootloader from a running app" on hardware

**Status: wired into the production test flow; needs a real-hardware run.**
This is no longer a standalone check — it's now two steps of the jig's normal
test sequence (`production_tools/programAndTest`, `env:lilygo-t-display`), so
every board that goes through the jig exercises this path:

1. **`TEST_FW_BOOTSTRAP`** (`testFwBootstrap()`): `test_host.py`'s
   `upload_firmware()` UPDI-flashes a **fixed, checked-in** old image
   (bootloader + `FW_VERSION=0` app) —
   `production_tools/programAndTest/factory_test_images/old_firmware_fw0.hex`
   (see its README for how it was built/regenerated) — establishing "a board
   that already has the bootloader and is running an old app" as the test's
   starting state, instead of the old `fb_legacy_no_bootloader` flash. Then
   `FIRMWARE_PHASE_VALIDATE_OLD` confirms the old app is running and reports
   `REG_FW_VERSION == OLD_FW_VERSION_FOR_TEST` (0).
2. **`TEST_FW_I2C_UPDATE`** (`testFwI2cUpdate()`): `FIRMWARE_PHASE_BOOTLOAD_UPDATE`
   calls `FaderBuddyBootloader::updateFirmware()` against the **pre-baked
   current app image** (`include/fader_app_image.h`, regenerated every build
   by `tools/pre_embed_app_image.py`, now wired into `env:lilygo-t-display` as
   well as the standalone `env:bootloader_test`) — driving `REG_ENTER_BOOTLOADER`
   entry from the *running old app*, erase, stream, whole-image CRC verify,
   `RUN_APP`, and `REG_FW_VERSION` re-check — then the jig continues into the
   existing `TEST_DEBUG_LED` / diagnostics / touch steps against the
   freshly-updated app.

The LCD's top-right corner shows the fader's live `Pn Fn` (protocol/firmware
version) readout throughout, so a bench run can visually confirm the version
flip from the old to the current build as `TEST_FW_I2C_UPDATE` runs.

- **Acceptance:** run the jig on a bench board and confirm `TEST_FW_BOOTSTRAP`
  then `TEST_FW_I2C_UPDATE` both pass end-to-end (old-firmware UPDI flash →
  validated old version → I2C bootloader entry from the running old app →
  update → new version confirmed) and the rest of the test suite passes
  against the updated app. **Not yet run on hardware** — this session only
  built, merged, and compile-checked the pieces; a bench run with a real board
  + jig is still needed.

Files: `production_tools/programAndTest/src/main.cpp` (`testFwBootstrap()`,
`testFwI2cUpdate()`, `FIRMWARE_PHASE_VALIDATE_OLD`/`FIRMWARE_PHASE_BOOTLOAD_UPDATE`),
`production_tools/programAndTest/test_host.py` (`upload_firmware()`),
`production_tools/programAndTest/factory_test_images/`,
`firmware/src/main.cpp` (search `i2c_enter_bootloader_request`,
`BL_ENTRY_TOKEN`), `production_tools/programAndTest/src/fader_buddy_bootloader.cpp`
(`enterBootloader()` / `waitForMarker()` / `updateFirmware()`).

## P1 — ESPHome host-side firmware packaging + update flow (design §11)

The biggest remaining feature. Only the protocol constants exist in
`esphome/components/fader_buddy/`; the update state machine does not. Implement
per ABOUT_I2C_BOOTLOADER.md §11:

- Embed the offset app image + `REG_FW_VERSION` via `__init__.py` `to_code()`
  (emit the ~11.5 KB blob **once**, shared across `MULTI_CONF` instances).
- Add a manual `fader_buddy.update_firmware` action driving: read version → (if
  mismatch) `REG_ENTER_BOOTLOADER` → wait for marker → `ERASE_APP` → stream pages
  → whole-image `GET_VERSION_CRC16` verify → `RUN_APP` → re-read `REG_FW_VERSION`.
  Reuse `write_with_retry_()`.
- Config surface: `firmware_image:`, `autoupdate_firmware:` (default false),
  `max_update_attempts:`.
- Guard rails: update only on version mismatch; per-address+version attempt cap
  in `ESPPreferences`; defer while `MODE_INPUT_ACTIVE`; one fader at a time.
- Port the bootloader client logic from the jig
  (`production_tools/programAndTest/src/fader_buddy_bootloader.cpp`) — it's the
  reference implementation of the sequence.
- **Acceptance:** ESPHome updates a mismatched fader via the manual action;
  refuses to reflash a fader already at the packaged version; attempt cap stops
  retries and surfaces a failure status.

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
