# `old_app_fw0.hex`

A **fixed, checked-in** snapshot of the FaderBuddy application, built with
`FW_VERSION` forced to `0` and linked at the boot offset (`BL_APP_START`). It
stands in for "an old application already in the field." It is also built with
its heartbeat LED blink period doubled (half the normal blink rate), so a board
still running this old image is obvious at a glance.

This file holds the application **only**. The bootloader is not part of it.

## How the jig uses it

`TEST_FW_BOOTSTRAP` UPDI-flashes the DUT with the **current** bootloader and
its fuses only (see `upload_firmware()` in `../test_host.py`). The bootloader
is built from source (`env:fb_bootloader_only`) when `test_host.py` starts.
The jig then installs this fixed old application through that bootloader over
I2C - it is embedded in the jig firmware as `FADER_OLD_APP_IMAGE` by
`../tools/generate_app_image.py`, so re-build the jig after regenerating it.

`TEST_FW_I2C_UPDATE` then drives `REG_ENTER_BOOTLOADER` from that running old
app and updates it to the current application over I2C, exactly the way a real
in-field update works.

The old app goes over I2C rather than UPDI only because it is much faster:
serial UPDI is bound by USB round-trip latency per flash page.

Building the bootloader fresh for every test session is deliberate: a checked-in image of
the bootloader would silently ship a **stale bootloader** on every production
board flashed after a bootloader change. Only the application half is frozen,
because the test needs an old application to update away from.

## When to regenerate

Ordinary application changes must **not** touch this file -- the point is that
it stays fixed, so the test keeps exercising a real old-to-new transition.

Regenerate it only when the old starting state itself must change. The one case
that forces it is a **`BL_BOOTEND` change**: this image is linked at
`BL_APP_START`, so a different boot/app split leaves it at an address the new
bootloader will not jump to. Last regenerated for `BOOTEND = 0x06`. This image
is one of the hardcoded copies of the flash geometry listed under "Hardcoded
copies of the flash geometry" in `ABOUT_I2C_BOOTLOADER.md`.

## How it was generated

```bash
source ~/.platformio/penv/bin/activate
cd <repo root>   # where the root platformio.ini lives

# DEBUG_LED_BLINK_PERIOD_MS is doubled so the heartbeat LED blinks at half the
# normal rate -- a visual cue that a board is running this old image.
PLATFORMIO_BUILD_FLAGS="-DFW_VERSION=0 -DDEBUG_LED_BLINK_PERIOD_MS=1024" \
  pio run -e fb_app_only

cp .pio/build/fb_app_only/firmware.hex \
  production_tools/programAndTest/factory_test_images/old_app_fw0.hex
```

`FW_VERSION=0` is baked in via the `#ifndef FW_VERSION` guard in
`firmware/src/shared/i2c_data.h`, which otherwise defaults to the real current
version. The jig firmware's `OLD_FW_VERSION_FOR_TEST` constant
(`../src/main.cpp`) must match whatever value is used here.
