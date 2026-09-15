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
#
# PlatformIO pre-build hook: (re)generate the embedded FaderBuddy application
# image header before compiling the jig.
import os
import subprocess
import sys

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

project_dir = env["PROJECT_DIR"]  # noqa: F821
generator = os.path.join(project_dir, "tools", "generate_app_image.py")

print("Embedding FaderBuddy application image (generate_app_image.py)...")
subprocess.run([sys.executable, generator], check=True)
