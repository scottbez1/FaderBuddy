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

Builds the fb_app_only PlatformIO environment (unless --no-build) and extracts
the application bytes from the resulting Intel-hex (see fb_image). The app's
FW_VERSION is baked into the last 2 bytes of the image at a fixed address (see
bootloader_protocol.h BL_APP_META_ADDR / main.cpp FW_VERSION_FOOTER), so the
ESPHome component reads it straight from there and this script does not need to
pass it along separately.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fb_image import build_offset_app, load_app_image  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True, help="output .bin path")
    p.add_argument("--no-build", action="store_true", help="use the existing hex, don't rebuild")
    args = p.parse_args()

    if not args.no_build:
        build_offset_app()

    image = load_app_image()
    fw_version = (image[-2] << 8) | image[-1]
    Path(args.output).write_bytes(image)
    print("Wrote %s: %u bytes, FW_VERSION=%u (from image footer)"
          % (args.output, len(image), fw_version), flush=True)


if __name__ == "__main__":
    main()
