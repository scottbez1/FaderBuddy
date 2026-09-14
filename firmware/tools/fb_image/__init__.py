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
"""Shared helpers for building and packaging the FaderBuddy offset application
image.

The flash geometry and the firmware version are read out of the C headers that
already define them, so no Python tool carries its own copy of BL_BOOTEND, the
page size or FW_VERSION. Consumers:

- `firmware/tools/export_app_image.py` - raw .bin for the ESPHome component
- `production_tools/programAndTest/tools/generate_app_image.py` - C header for the jig
- `production_tools/programAndTest/test_host.py` - BOOTEND/APPEND for the UPDI flash
- `ci/firmware/package_release.py` - release asset naming

Import it by putting `firmware/tools` on sys.path:

    sys.path.insert(0, str(REPO_ROOT / "firmware" / "tools"))
    from fb_image import BL_APP_START, load_app_image
"""

import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SHARED_DIR = REPO_ROOT / "firmware" / "src" / "shared"
BOOTLOADER_PROTOCOL_H = SHARED_DIR / "bootloader_protocol.h"
I2C_DATA_H = SHARED_DIR / "i2c_data.h"

OFFSET_ENV = "fb_app_only"
BOOTLOADER_ENV = "fb_bootloader_only"
OFFSET_HEX = REPO_ROOT / ".pio" / "build" / OFFSET_ENV / "firmware.hex"
BOOTLOADER_HEX = REPO_ROOT / ".pio" / "build" / BOOTLOADER_ENV / "firmware.hex"


def parse_define(path, name):
    """Parse a simple `#define NAME (value)` integer out of a C header."""
    text = Path(path).read_text()
    m = re.search(r"#define\s+%s\s+\(?\s*(0x[0-9A-Fa-f]+|\d+)" % re.escape(name), text)
    if not m:
        raise RuntimeError("could not find %s in %s" % (name, path))
    return int(m.group(1), 0)


# Flash geometry, straight from bootloader_protocol.h.
BL_BOOTEND = parse_define(BOOTLOADER_PROTOCOL_H, "BL_BOOTEND")
BL_FLASH_SIZE = parse_define(BOOTLOADER_PROTOCOL_H, "BL_FLASH_SIZE")
BL_PAGE_SIZE = parse_define(BOOTLOADER_PROTOCOL_H, "BL_PAGE_SIZE")
BL_APPEND = parse_define(BOOTLOADER_PROTOCOL_H, "BL_APPEND")
BL_APP_START = BL_BOOTEND * 256


def firmware_version():
    """The app's FW_VERSION as (major, minor), from i2c_data.h."""
    return (parse_define(I2C_DATA_H, "FW_VERSION_MAJOR"),
            parse_define(I2C_DATA_H, "FW_VERSION_MINOR"))


def firmware_version_u16():
    """FW_VERSION as the packed u16 the firmware reports at REG_FW_VERSION."""
    major, minor = firmware_version()
    return (major << 8) | minor


def crc16_ccitt(data):
    """CRC16-CCITT (poly 0x1021, init 0xFFFF) - the bootloader's bl_crc16_update()."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_env(env):
    """Build one PlatformIO environment from the root project."""
    print("Building %s ..." % env, flush=True)
    subprocess.run(
        [sys.executable, "-m", "platformio", "run", "-e", env],
        cwd=str(REPO_ROOT), check=True,
    )


def build_offset_app():
    build_env(OFFSET_ENV)


def load_app_image(hex_path=None):
    """Extract the offset application bytes from an Intel-hex.

    Takes everything from BL_APP_START up, with gaps read as 0xFF (erased
    flash), padded up to a whole flash page.
    """
    from intelhex import IntelHex  # ships with pymcuprog

    hex_path = Path(hex_path or OFFSET_HEX)
    if not hex_path.exists():
        raise SystemExit("offset app hex not found: %s (build %s first)"
                         % (hex_path, OFFSET_ENV))
    ih = IntelHex(str(hex_path))
    minaddr, maxaddr = ih.minaddr(), ih.maxaddr()
    if minaddr < BL_APP_START:
        raise RuntimeError("hex has data below the boot offset (0x%04X < 0x%04X)"
                           % (minaddr, BL_APP_START))
    ih.padding = 0xFF
    image = bytearray(ih.tobinarray(start=BL_APP_START, end=maxaddr))
    if len(image) % BL_PAGE_SIZE:
        image += b"\xFF" * (BL_PAGE_SIZE - (len(image) % BL_PAGE_SIZE))
    if BL_APP_START + len(image) > BL_FLASH_SIZE:
        raise RuntimeError("image overflows flash")
    return bytes(image)
