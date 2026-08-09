#!/usr/bin/env python3
# Copyright 2026 Scott Bezek
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Export the FaderBuddy offset application image as a raw .bin, for the
ESPHome component's `firmware_image:` config (I2C-bootloader updates).

Builds the fb_app_only PlatformIO environment (unless --no-build) and
extracts the exact APPCODE bytes from the resulting Intel-hex, starting at
the boot offset and padded to a whole number of flash pages with 0xFF. The
app's FW_VERSION is baked into the last 2 bytes of the image at a fixed
address (see bootloader_protocol.h BL_APP_META_ADDR / main.cpp
FW_VERSION_FOOTER) -- the ESPHome component reads it straight from there, so
this script does not need to (and cannot) pass it along separately.

This mirrors production_tools/programAndTest/tools/generate_app_image.py,
which does the same extraction for the jig but emits a C header instead of a
raw binary.
"""

import argparse
import subprocess
import sys
from pathlib import Path

# firmware/src/shared/bootloader_protocol.h constants (kept trivially in sync).
FLASH_START = 0x0800   # BL_APP_START (BOOTEND 0x08 * 256)
FLASH_SIZE = 16384     # BL_FLASH_SIZE
PAGE_SIZE = 64         # BL_PAGE_SIZE

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]  # tools -> firmware -> repo root
OFFSET_ENV = "fb_app_only"
OFFSET_HEX = REPO_ROOT / ".pio" / "build" / OFFSET_ENV / "firmware.hex"


def build_offset_app():
    print("Building %s ..." % OFFSET_ENV, flush=True)
    subprocess.run(
        [sys.executable, "-m", "platformio", "run", "-e", OFFSET_ENV],
        cwd=str(REPO_ROOT), check=True,
    )


def load_app_image():
    from intelhex import IntelHex  # ships with pymcuprog
    ih = IntelHex(str(OFFSET_HEX))
    minaddr, maxaddr = ih.minaddr(), ih.maxaddr()
    if minaddr < FLASH_START:
        raise RuntimeError("hex has data below the boot offset (0x%04X < 0x%04X)"
                           % (minaddr, FLASH_START))
    # Extract [FLASH_START, maxaddr]; gaps read as 0xFF (erased flash).
    ih.padding = 0xFF
    image = bytearray(ih.tobinarray(start=FLASH_START, end=maxaddr))
    # Pad up to a whole page.
    if len(image) % PAGE_SIZE:
        image += b"\xFF" * (PAGE_SIZE - (len(image) % PAGE_SIZE))
    if FLASH_START + len(image) > FLASH_SIZE:
        raise RuntimeError("image overflows flash")
    return bytes(image)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True, help="output .bin path")
    p.add_argument("--no-build", action="store_true", help="use the existing hex, don't rebuild")
    args = p.parse_args()

    if not args.no_build:
        build_offset_app()
    if not OFFSET_HEX.exists():
        raise SystemExit("offset app hex not found: %s (build fb_app_only first)" % OFFSET_HEX)

    image = load_app_image()
    fw_version = (image[-2] << 8) | image[-1]
    Path(args.output).write_bytes(image)
    print("Wrote %s: %u bytes, FW_VERSION=%u (from image footer)"
          % (args.output, len(image), fw_version), flush=True)


if __name__ == "__main__":
    main()
