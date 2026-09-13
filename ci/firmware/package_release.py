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
"""Package the FaderBuddy application image as a release asset.

Builds the offset application (via firmware/tools/export_app_image.py), names the
resulting .bin after the FW_VERSION baked into it, and reports its sha256 -- the
value that goes into KNOWN_FIRMWARE in the ESPHome component.

Release assets are attached to a tag following the same scheme as the electronics
artifacts (see ci/util/rev_info.py): `releases/firmware/v<major>.<minor>`. Pass
--expect-version to assert the tag and the firmware's own FW_VERSION agree, so a
mislabelled asset is caught at release time rather than by whoever consumes it.
"""

import argparse
import hashlib
import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SHARED_I2C_DATA = REPO_ROOT / "firmware" / "src" / "shared" / "i2c_data.h"
EXPORT_TOOL = REPO_ROOT / "firmware" / "tools" / "export_app_image.py"

RELEASE_TAG_PREFIX = "releases/firmware/"


def parse_define(path, name):
    text = Path(path).read_text()
    m = re.search(r"#define\s+%s\s+\(?\s*(0x[0-9A-Fa-f]+|\d+)" % re.escape(name), text)
    if not m:
        raise RuntimeError("could not find %s in %s" % (name, path))
    return int(m.group(1), 0)


def firmware_version():
    major = parse_define(SHARED_I2C_DATA, "FW_VERSION_MAJOR")
    minor = parse_define(SHARED_I2C_DATA, "FW_VERSION_MINOR")
    return major, minor


def version_from_tag(ref):
    """'refs/tags/releases/firmware/v1.3' or 'releases/firmware/v1.3' -> '1.3'."""
    tag = ref[len("refs/tags/"):] if ref.startswith("refs/tags/") else ref
    if not tag.startswith(RELEASE_TAG_PREFIX):
        raise SystemExit(
            "tag %r does not start with %r" % (tag, RELEASE_TAG_PREFIX))
    return tag[len(RELEASE_TAG_PREFIX):].lstrip("v")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--output-dir", default=str(REPO_ROOT / "firmware" / "build"),
                   help="directory to write the named .bin into")
    p.add_argument("--no-build", action="store_true",
                   help="use the existing fb_app_only hex, don't rebuild")
    p.add_argument("--expect-version",
                   help="tag or bare version the image must match, e.g. "
                        "'refs/tags/releases/firmware/v1.3' or '1.3'")
    args = p.parse_args()

    major, minor = firmware_version()
    version = "%u.%u" % (major, minor)

    if args.expect_version:
        want = version_from_tag(args.expect_version)
        if want != version:
            raise SystemExit(
                "Tag asks for firmware v%s but FW_VERSION in i2c_data.h is v%s.\n"
                "Bump FW_VERSION_MAJOR/FW_VERSION_MINOR to match the tag (or retag), "
                "so the released asset is not mislabelled." % (want, version))

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / ("fader_buddy_app_v%s.bin" % version)

    cmd = [sys.executable, str(EXPORT_TOOL), "--output", str(out_path)]
    if args.no_build:
        cmd.append("--no-build")
    subprocess.run(cmd, cwd=str(REPO_ROOT), check=True)

    data = out_path.read_bytes()
    sha256 = hashlib.sha256(data).hexdigest()

    print()
    print("Release asset: %s" % out_path.name)
    print("  version : %s" % version)
    print("  size    : %u bytes" % len(data))
    print("  sha256  : %s" % sha256)
    print("  tag     : %sv%s" % (RELEASE_TAG_PREFIX, version))
    print()
    print("Add to KNOWN_FIRMWARE in esphome/components/fader_buddy/__init__.py:")
    print('    "%s": "%s",' % (version, sha256))

    # Consumed by the workflow to name the release and set outputs.
    if "GITHUB_OUTPUT" in os.environ:
        with Path(os.environ["GITHUB_OUTPUT"]).open("a") as f:
            f.write("version=%s\n" % version)
            f.write("sha256=%s\n" % sha256)
            f.write("asset_path=%s\n" % out_path)
            f.write("asset_name=%s\n" % out_path.name)


if __name__ == "__main__":
    main()
