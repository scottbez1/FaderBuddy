# PlatformIO extra script for env:fb_bootloader_only: cap the linker's flash
# region at BL_APP_START, so a bootloader that outgrows the boot section fails
# the build itself (locally and in CI) with "region `text' overflowed".
Import("env")  # noqa: F821 - provided by PlatformIO/SCons

import os
import sys

sys.path.insert(0, os.path.join(env.subst("$PROJECT_DIR"), "firmware", "tools"))  # noqa: F821
from fb_image import BL_APP_START  # noqa: E402

env.Append(LINKFLAGS=["-Wl,--defsym=__TEXT_REGION_LENGTH__=%d" % BL_APP_START])  # noqa: F821
