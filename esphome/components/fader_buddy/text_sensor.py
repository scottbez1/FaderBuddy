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

"""REMOVED in 0.5.0: `text_sensor: platform: fader_buddy`.

Kept only so an old config fails with migration instructions, rather than
ESPHome's generic "platform not found".
"""

import esphome.config_validation as cv

CONFIG_SCHEMA = cv.invalid(
    "`text_sensor: platform: fader_buddy` was removed in fader_buddy 0.5.0. The hub creates "
    "its diagnostic text sensors itself - delete this text_sensor block, and if you were "
    "overriding the name or icon, move that onto the hub's own `serial_number:` key instead."
)
