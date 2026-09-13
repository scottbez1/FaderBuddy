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

"""DEPRECATED: `text_sensor: platform: fader_buddy`.

The hub creates its own diagnostic text sensors now, so this platform exists
only to keep existing configs working. It will be removed in component 0.5.0.

To migrate, delete the whole `text_sensor:` entry. The hub already provides a
serial number sensor; move any `name:`/`id:`/`icon:` onto the hub's own
`serial_number:` key if you were overriding them:

    fader_buddy:
      - id: my_fader
        serial_number:
          name: "My Fader Serial"
"""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_FADER_BUDDY_ID, CONF_SERIAL_NUMBER, FaderBuddy

DEPENDENCIES = ["fader_buddy"]

_LOGGER = logging.getLogger(__name__)


def _warn_deprecated(config):
    _LOGGER.warning(
        "`text_sensor: platform: fader_buddy` is deprecated and will be removed in "
        "fader_buddy 0.5.0. The hub creates its diagnostic text sensors itself now - "
        "delete this text_sensor block, and if you were overriding the name or icon, "
        "move that onto the hub's own `serial_number:` key instead."
    )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_FADER_BUDDY_ID): cv.use_id(FaderBuddy),
            cv.Optional(CONF_SERIAL_NUMBER): text_sensor.text_sensor_schema(
                entity_category="diagnostic",
            ),
        }
    ),
    _warn_deprecated,
)


async def to_code(config):
    # The hub skips creating its own sensor for anything claimed here (see
    # _claimed_by_legacy_platform in __init__.py), so this stays the only one.
    parent = await cg.get_variable(config[CONF_FADER_BUDDY_ID])

    if CONF_SERIAL_NUMBER in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SERIAL_NUMBER])
        cg.add(parent.set_serial_text_sensor(sens))
