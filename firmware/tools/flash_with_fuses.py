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
"""Flash one or more Intel-hex images (and optionally the BOOTEND/APPEND fuses)
to an ATtiny1616 over serial UPDI using pymcuprog.

Used by the bootloader-related PlatformIO environments (see platformio.ini) to
install the boot section, the offset application, and the correct fuses in a
single UPDI upload. With --erase the first --hex triggers a full chip erase
(used when installing the bootloader / doing a factory flash); additional --hex
files are always written without a chip erase (disjoint flash regions, page
erases happen per write). Without --erase nothing is chip-erased, so you can
re-flash just the offset application on top of an existing bootloader.
"""

import argparse
import os
import subprocess
import sys

DEVICE = "attiny1616"


def pymcuprog_cmd():
    """pymcuprog installs a console-script (no runnable __main__ module), so
    invoke the executable that sits next to the current interpreter."""
    exe = os.path.join(os.path.dirname(sys.executable), "pymcuprog")
    return [exe] if os.path.exists(exe) else ["pymcuprog"]

# ATtiny1616 fuse byte numbers (see ATtiny1614/16/17 datasheet + megaTinyCore).
FUSE_APPEND = 7
FUSE_BOOTEND = 8


def parse_int(value):
    return int(value, 0)


def pymcuprog(args, port):
    cmd = pymcuprog_cmd() + args[:1] + [
        "-d", DEVICE,
        "-t", "uart",
        "-u", port,
    ] + args[1:]
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True, help="serial UPDI port")
    p.add_argument(
        "--hex",
        action="append",
        default=[],
        required=True,
        dest="hexes",
        help="Intel-hex image to write (repeatable; first one erases the chip)",
    )
    p.add_argument("--bootend", type=parse_int, default=None,
                   help="BOOTEND fuse value (256-byte units), e.g. 0x08")
    p.add_argument("--append", type=parse_int, default=None,
                   help="APPEND fuse value (256-byte units), e.g. 0x00")
    p.add_argument("--erase", action="store_true",
                   help="chip-erase before writing the first hex (bootloader install / factory)")
    args = p.parse_args()

    for i, hexfile in enumerate(args.hexes):
        write_args = ["write", "-f", hexfile, "--verify"]
        if i == 0 and args.erase:
            write_args.append("--erase")
        pymcuprog(write_args, args.port)

    if args.append is not None:
        pymcuprog(["write", "-m", "fuses", "-o", str(FUSE_APPEND),
                   "-l", str(args.append)], args.port)
    if args.bootend is not None:
        pymcuprog(["write", "-m", "fuses", "-o", str(FUSE_BOOTEND),
                   "-l", str(args.bootend)], args.port)

    print("Flash complete.", flush=True)


if __name__ == "__main__":
    main()
