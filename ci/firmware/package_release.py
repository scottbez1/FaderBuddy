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

Release assets are attached to the tag `releases/firmware/v<major>.<minor>`. Pass
--expect-version to assert the tag and the firmware's own FW_VERSION agree, so a
mislabelled asset is caught at release time rather than by whoever consumes it.

Releases are cut by hand -- CI has no write access to the repo -- so with
--expect-version this also emits the `gh release create` invocation and the release
body to walk through that, into $GITHUB_STEP_SUMMARY when running under Actions.
"""

import argparse
import hashlib
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
EXPORT_TOOL = REPO_ROOT / "firmware" / "tools" / "export_app_image.py"

sys.path.insert(0, str(REPO_ROOT / "firmware" / "tools"))
from fb_image import firmware_version  # noqa: E402

RELEASE_TAG_PREFIX = "releases/firmware/"


def version_from_tag(ref):
    """'refs/tags/releases/firmware/v1.3' or 'releases/firmware/v1.3' -> '1.3'."""
    tag = ref[len("refs/tags/"):] if ref.startswith("refs/tags/") else ref
    if not tag.startswith(RELEASE_TAG_PREFIX):
        raise SystemExit(
            "tag %r does not start with %r" % (tag, RELEASE_TAG_PREFIX))
    return tag[len(RELEASE_TAG_PREFIX):].lstrip("v")


def release_instructions(version, sha256, asset_name):
    """Markdown walking through cutting the release by hand.

    Rendered into the job summary under Actions; printed to the log otherwise. The
    fenced blocks are meant to be copy-pasted as-is, hence the ```` fence around the
    release body, which itself contains ``` blocks.
    """
    run_url = "%s/%s/actions/runs/%s" % (
        os.environ.get("GITHUB_SERVER_URL", "https://github.com"),
        os.environ.get("GITHUB_REPOSITORY", "scottbez1/motorFader"),
        os.environ.get("GITHUB_RUN_ID", "<run id>"))
    facts = (
        "| | |\n"
        "|---|---|\n"
        "| Version | `%s` |\n"
        "| Asset | `%s` |\n"
        "| sha256 | `%s` |\n" % (version, asset_name, sha256))
    return """## Firmware v{version} ready to release

{facts}| Tag | `{prefix}v{version}` |

Download the `firmware` artifact from [this run]({run_url}), then:

```bash
unzip -j firmware.zip '*/{asset_name}' -d .
sha256sum -c <<< '{sha256}  {asset_name}'
gh release create '{prefix}v{version}' '{asset_name}' \\
  --title 'Firmware v{version}' --notes-file release-notes.md
```

### release-notes.md

````markdown
ATtiny1616 application image for I2C-bootloader updates.

{facts}
Add to `KNOWN_FIRMWARE` in `esphome/components/fader_buddy/__init__.py`:

```python
    "{version}": "{sha256}",
```

Then in ESPHome:

```yaml
fader_buddy:
  - id: fader0
    firmware: "{version}"
```
````
""".format(version=version, sha256=sha256, asset_name=asset_name, facts=facts,
           prefix=RELEASE_TAG_PREFIX, run_url=run_url)


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

    if args.expect_version:
        instructions = release_instructions(version, sha256, out_path.name)
        print()
        print(instructions)
        summary = os.environ.get("GITHUB_STEP_SUMMARY")
        if summary:
            with Path(summary).open("a") as f:
                f.write(instructions)


if __name__ == "__main__":
    main()
