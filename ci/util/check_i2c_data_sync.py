#!/usr/bin/env python3
#
# Verifies that every copy of a shared source file is byte-for-byte identical
# to its canonical copy. Run with --fix to update the copies instead.
#
# Two kinds of shared source:
#   - the protocol headers, canonical under firmware/src/shared/, copied into
#     the host library and the ESPHome component;
#   - the host library (hostLib/src/), copied into the ESPHome component,
#     because an ESPHome external component only compiles files inside its
#     own directory.

import argparse
import difflib
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

SHARED = REPO_ROOT / 'firmware/src/shared'
HOSTLIB = REPO_ROOT / 'hostLib/src'
ESPHOME = REPO_ROOT / 'esphome/components/fader_buddy'

# Host library files the ESPHome component compiles. Not the Arduino adapter
# (the component has its own) or the umbrella header.
HOSTLIB_FOR_ESPHOME = [
    'fader_buddy_platform.h',
    'fader_buddy_protocol.h',
    'fader_buddy_protocol.cpp',
    'fader_buddy_device.h',
    'fader_buddy_device.cpp',
    'fader_buddy_updater.h',
    'fader_buddy_updater.cpp',
    'fader_buddy_host.h',
    'fader_buddy_host.cpp',
]

# canonical -> copies that must match it exactly
SHARED_FILES = {
    SHARED / 'i2c_data.h': [
        ESPHOME / 'i2c_data.h',
        HOSTLIB / 'i2c_data.h',
    ],
    SHARED / 'bootloader_protocol.h': [
        ESPHOME / 'bootloader_protocol.h',
        HOSTLIB / 'bootloader_protocol.h',
    ],
}
for name in HOSTLIB_FOR_ESPHOME:
    SHARED_FILES[HOSTLIB / name] = [ESPHOME / name]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fix', action='store_true',
                        help='copy each canonical file over its copies instead of checking')
    args = parser.parse_args()

    status = 0
    for canonical, copies in SHARED_FILES.items():
        canonical_text = canonical.read_text()
        for copy in copies:
            copy_text = copy.read_text() if copy.exists() else ''
            if copy_text == canonical_text:
                continue
            if args.fix:
                shutil.copyfile(canonical, copy)
                print(f'updated {copy.relative_to(REPO_ROOT)}')
                continue
            sys.stdout.writelines(difflib.unified_diff(
                canonical_text.splitlines(keepends=True), copy_text.splitlines(keepends=True),
                fromfile=str(canonical.relative_to(REPO_ROOT)), tofile=str(copy.relative_to(REPO_ROOT)),
            ))
            print(f'error: {copy.relative_to(REPO_ROOT)} has drifted from '
                  f'{canonical.relative_to(REPO_ROOT)} (run {Path(__file__).name} --fix)', file=sys.stderr)
            status = 1

    return status


if __name__ == '__main__':
    sys.exit(main())
