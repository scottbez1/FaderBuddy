#!/usr/bin/env python3
#
# Verifies that every copy of the I2C protocol header is byte-for-byte
# identical to the canonical copy in firmware/src/shared/i2c_data.h.

import difflib
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

CANONICAL = REPO_ROOT / 'firmware/src/shared/i2c_data.h'

COPIES = [
    REPO_ROOT / 'esphome/components/fader_buddy/i2c_data.h',
]

def main():
    canonical_lines = CANONICAL.read_text().splitlines(keepends=True)

    status = 0
    for copy in COPIES:
        copy_lines = copy.read_text().splitlines(keepends=True)
        diff = list(difflib.unified_diff(
            canonical_lines, copy_lines,
            fromfile=str(CANONICAL), tofile=str(copy),
        ))
        if diff:
            sys.stdout.writelines(diff)
            print(f'error: {copy} has drifted from {CANONICAL}', file=sys.stderr)
            status = 1

    return status

if __name__ == '__main__':
    sys.exit(main())
