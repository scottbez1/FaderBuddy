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

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_FADER_BUDDY_ID, FaderBuddy

DEPENDENCIES = ["fader_buddy"]

CONF_SERIAL_NUMBER = "serial_number"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_FADER_BUDDY_ID): cv.use_id(FaderBuddy),
        cv.Optional(CONF_SERIAL_NUMBER): text_sensor.text_sensor_schema(
            entity_category="diagnostic",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_FADER_BUDDY_ID])

    if CONF_SERIAL_NUMBER in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SERIAL_NUMBER])
        cg.add(parent.set_serial_text_sensor(sens))
