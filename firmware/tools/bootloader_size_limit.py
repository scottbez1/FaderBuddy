# PlatformIO extra script for env:fb_bootloader_only: hold the bootloader to
# the boot section (BL_APP_START bytes, from bootloader_protocol.h via fb_image).
#
# - The linker's flash region is capped there, so a bootloader that outgrows
#   the boot section fails the build itself - locally and in CI alike - with
#   "region `text' overflowed by N bytes". Without this the link succeeds
#   against the chip's full 16 KB, and the overlap only shows up when the app
#   image is installed on top of it.
# - The size readout ("Flash: used N bytes from M") is reported against the
#   boot section rather than the whole chip, so headroom is visible every build.
Import("env")  # noqa: F821 - provided by PlatformIO/SCons

import os
import sys

sys.path.insert(0, os.path.join(env.subst("$PROJECT_DIR"), "firmware", "tools"))  # noqa: F821
from fb_image import BL_APP_START  # noqa: E402

env.Append(LINKFLAGS=["-Wl,--defsym=__TEXT_REGION_LENGTH__=%d" % BL_APP_START])  # noqa: F821
env.BoardConfig().update("upload.maximum_size", BL_APP_START)  # noqa: F821
