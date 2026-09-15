#!/usr/bin/env python3
#
# Verifies that every copy of a shared header is byte-for-byte identical to its
# canonical copy under firmware/src/shared/.

import difflib
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

# canonical -> copies that must match it exactly
SHARED_HEADERS = {
    REPO_ROOT / 'firmware/src/shared/i2c_data.h': [
        REPO_ROOT / 'esphome/components/fader_buddy/i2c_data.h',
    ],
    REPO_ROOT / 'firmware/src/shared/bootloader_protocol.h': [
        REPO_ROOT / 'esphome/components/fader_buddy/bootloader_protocol.h',
    ],
}

def main():
    status = 0
    for canonical, copies in SHARED_HEADERS.items():
        canonical_lines = canonical.read_text().splitlines(keepends=True)
        for copy in copies:
            copy_lines = copy.read_text().splitlines(keepends=True)
            diff = list(difflib.unified_diff(
                canonical_lines, copy_lines,
                fromfile=str(canonical), tofile=str(copy),
            ))
            if diff:
                sys.stdout.writelines(diff)
                print(f'error: {copy} has drifted from {canonical}', file=sys.stderr)
                status = 1

    return status

if __name__ == '__main__':
    sys.exit(main())
