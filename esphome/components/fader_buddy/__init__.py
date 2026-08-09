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

from pathlib import Path

from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_MODE

MULTI_CONF = True
DEPENDENCIES = ["i2c"]

# firmware/src/shared/bootloader_protocol.h BL_PAGE_SIZE -- the packaged image must be
# an exact multiple of this (see firmware/tools/export_app_image.py, which produces it).
BL_PAGE_SIZE = 64


def _crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021, init 0xFFFF), matching bl_crc16_update() in
    bootloader_protocol.h -- used by the bootloader's own whole-image verify."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# Cache of already-embedded firmware images, keyed by resolved path, so multiple
# fader_buddy instances pointing at the same firmware_image share one emitted
# array instead of duplicating a ~14 KB blob per instance (MULTI_CONF).
_firmware_image_cache: dict[Path, tuple[str, int, int, int]] = {}


def _get_or_emit_firmware_image(path: Path) -> tuple[str, int, int, int]:
    """Returns (symbol_name, length, crc16, fw_version), emitting the array once."""
    resolved = path.resolve()
    cached = _firmware_image_cache.get(resolved)
    if cached is not None:
        return cached

    data = resolved.read_bytes()
    if len(data) == 0 or len(data) % BL_PAGE_SIZE != 0:
        raise cv.Invalid(
            f"Firmware image '{path}' must be a non-empty multiple of {BL_PAGE_SIZE} bytes "
            "-- the exact page-aligned APPCODE image produced by "
            "firmware/tools/export_app_image.py, not an Intel-hex or .elf file."
        )

    # Last 2 bytes are the FW_VERSION_FOOTER (bootloader_protocol.h BL_APP_META_ADDR),
    # baked in by the firmware build itself -- always in sync with what the flashed app
    # will report via REG_FW_VERSION, no separate YAML field to keep in sync by hand.
    fw_version = (data[-2] << 8) | data[-1]
    crc16 = _crc16_ccitt(data)

    symbol = f"fader_buddy_firmware_image_{len(_firmware_image_cache)}"
    array_body = ", ".join(str(b) for b in data)
    cg.add_global(
        cg.RawExpression(f"static const uint8_t {symbol}[{len(data)}] PROGMEM = {{{array_body}}}")
    )

    result = (symbol, len(data), crc16, fw_version)
    _firmware_image_cache[resolved] = result
    return result

fader_buddy_ns = cg.esphome_ns.namespace("fader_buddy")
FaderBuddy = fader_buddy_ns.class_("FaderBuddy", cg.PollingComponent, i2c.I2CDevice)

# Define haptic mode enum (in global namespace, shared with firmware)
HapticMode = cg.global_ns.enum("HapticMode")
HAPTIC_MODES = {
    "smooth": HapticMode.HAPTIC_NO_HAPTICS,
    "smooth_with_magnets": HapticMode.HAPTIC_SMOOTH_WITH_MAGNET_ENDS,
    "detents": HapticMode.HAPTIC_DETENTS,
}

CONF_ON_MANUAL_MOVE = "on_manual_move"
CONF_ON_RAW_POSITION_UPDATE = "on_raw_position_update"
CONF_ON_TOUCH_CHANGE = "on_touch_change"
CONF_ON_DOUBLE_TAP = "on_double_tap"
CONF_ON_FIRMWARE_UPDATE_RESULT = "on_firmware_update_result"
CONF_INVERT = "invert"
CONF_LAYER_HAPTICS = "layer_haptics"
CONF_LAYER = "layer"
CONF_DETENT_COUNT = "detent_count"
CONF_DETENT_STRENGTH = "detent_strength"
CONF_POSITION = "position"
CONF_VALUE_CHANGE_MIN_INTERVAL = "value_change_min_interval"
CONF_FIRMWARE_IMAGE = "firmware_image"
CONF_MAX_UPDATE_ATTEMPTS = "max_update_attempts"

# Schema for a single layer haptic configuration
LAYER_HAPTIC_SCHEMA = cv.Schema({
    cv.Required(CONF_LAYER): cv.int_range(min=0, max=7),
    cv.Required(CONF_MODE): cv.enum(HAPTIC_MODES, lower=True),
    cv.Optional(CONF_DETENT_COUNT, default=0): cv.int_range(min=0, max=15),
    cv.Optional(CONF_DETENT_STRENGTH, default=0): cv.int_range(min=0, max=7),
    cv.Optional(CONF_VALUE_CHANGE_MIN_INTERVAL, default="0ms"): cv.positive_time_period_milliseconds,
})

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(FaderBuddy),
        cv.Optional(CONF_ON_MANUAL_MOVE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_RAW_POSITION_UPDATE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_TOUCH_CHANGE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_DOUBLE_TAP): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_FIRMWARE_UPDATE_RESULT): automation.validate_automation(single=True),
        cv.Optional(CONF_INVERT, default=False): cv.boolean,
        cv.Optional(CONF_LAYER_HAPTICS): cv.ensure_list(LAYER_HAPTIC_SCHEMA),
        # Packaged application image for I2C-bootloader updates (manual action only --
        # see fader_buddy.update_firmware below). Optional: omit to disable updates
        # entirely for this fader. FW_VERSION is read from the image itself (see
        # bootloader_protocol.h BL_APP_META_ADDR), not a separate YAML field.
        cv.Optional(CONF_FIRMWARE_IMAGE): cv.file_,
        cv.Optional(CONF_MAX_UPDATE_ATTEMPTS, default=3): cv.int_range(min=1, max=20),
    })
    .extend(cv.polling_component_schema("50ms"))
    .extend(i2c.i2c_device_schema(0x20))  # default I2C address
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_INVERT in config:
        cg.add(var.set_invert(config[CONF_INVERT]))

    # Store initial layer haptic configurations (sent during setup)
    if CONF_LAYER_HAPTICS in config:
        for haptic_config in config[CONF_LAYER_HAPTICS]:
            layer = haptic_config[CONF_LAYER]
            mode = haptic_config[CONF_MODE]
            detent_count = haptic_config[CONF_DETENT_COUNT]
            detent_strength = haptic_config[CONF_DETENT_STRENGTH]
            min_interval = haptic_config[CONF_VALUE_CHANGE_MIN_INTERVAL]

            cg.add(var.store_initial_layer_haptic_config(
                layer, mode, detent_count, detent_strength
            ))
            cg.add(var.set_layer_value_change_min_interval(layer, min_interval))

    if CONF_ON_MANUAL_MOVE in config:
        await automation.build_automation(
            var.get_on_manual_move_trigger(), [(cg.uint8, "x"), (cg.uint8, "layer")], config[CONF_ON_MANUAL_MOVE]
        )

    if CONF_ON_RAW_POSITION_UPDATE in config:
        await automation.build_automation(
            var.get_on_raw_position_update_trigger(), [(cg.uint8, "x"), (cg.uint8, "layer")], config[CONF_ON_RAW_POSITION_UPDATE]
        )

    if CONF_ON_TOUCH_CHANGE in config:
        await automation.build_automation(
            var.get_on_touch_change_trigger(), [(cg.bool_, "x"), (cg.uint8, "layer")], config[CONF_ON_TOUCH_CHANGE]
        )

    if CONF_ON_DOUBLE_TAP in config:
        await automation.build_automation(
            var.get_on_double_tap_trigger(), [(cg.uint8, "layer")], config[CONF_ON_DOUBLE_TAP]
        )

    if CONF_ON_FIRMWARE_UPDATE_RESULT in config:
        await automation.build_automation(
            var.get_on_firmware_update_result_trigger(),
            [(cg.bool_, "success"), (cg.std_string, "message")],
            config[CONF_ON_FIRMWARE_UPDATE_RESULT],
        )

    if CONF_FIRMWARE_IMAGE in config:
        symbol, length, crc16, fw_version = _get_or_emit_firmware_image(config[CONF_FIRMWARE_IMAGE])
        cg.add(var.set_firmware_image(cg.RawExpression(symbol), length, crc16, fw_version))
        cg.add(var.set_max_update_attempts(config[CONF_MAX_UPDATE_ATTEMPTS]))


# Actions
SetActiveLayerAction = fader_buddy_ns.class_("SetActiveLayerAction", automation.Action)
RemoteMoveToAction = fader_buddy_ns.class_("RemoteMoveToAction", automation.Action)
SetLayerHapticConfigAction = fader_buddy_ns.class_("SetLayerHapticConfigAction", automation.Action)
RunSelfCalibrationAction = fader_buddy_ns.class_("RunSelfCalibrationAction", automation.Action)
UpdateFirmwareAction = fader_buddy_ns.class_("UpdateFirmwareAction", automation.Action)


@automation.register_action(
    "fader_buddy.set_active_layer",
    SetActiveLayerAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(FaderBuddy),
        cv.Required(CONF_LAYER): cv.templatable(cv.int_range(min=0, max=7)),
    })
)
async def set_active_layer_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    layer = await cg.templatable(config[CONF_LAYER], args, cg.uint8)
    cg.add(var.set_layer(layer))
    return var


@automation.register_action(
    "fader_buddy.remote_move_to",
    RemoteMoveToAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(FaderBuddy),
        cv.Required(CONF_POSITION): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_LAYER, default=0): cv.templatable(cv.int_range(min=0, max=7)),
    })
)
async def remote_move_to_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    position = await cg.templatable(config[CONF_POSITION], args, cg.uint8)
    cg.add(var.set_position(position))
    layer = await cg.templatable(config[CONF_LAYER], args, cg.uint8)
    cg.add(var.set_layer(layer))
    return var


@automation.register_action(
    "fader_buddy.set_layer_haptic_config",
    SetLayerHapticConfigAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(FaderBuddy),
        cv.Required(CONF_LAYER): cv.templatable(cv.int_range(min=0, max=7)),
        cv.Required(CONF_MODE): cv.templatable(cv.enum(HAPTIC_MODES, lower=True)),
        cv.Optional(CONF_DETENT_COUNT, default=0): cv.templatable(cv.int_range(min=0, max=15)),
        cv.Optional(CONF_DETENT_STRENGTH, default=0): cv.templatable(cv.int_range(min=0, max=7)),
    })
)
async def set_layer_haptic_config_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    layer = await cg.templatable(config[CONF_LAYER], args, cg.uint8)
    cg.add(var.set_layer(layer))
    mode = await cg.templatable(config[CONF_MODE], args, HapticMode)
    cg.add(var.set_mode(mode))
    detent_count = await cg.templatable(config[CONF_DETENT_COUNT], args, cg.uint8)
    cg.add(var.set_detent_count(detent_count))
    detent_strength = await cg.templatable(config[CONF_DETENT_STRENGTH], args, cg.uint8)
    cg.add(var.set_detent_strength(detent_strength))
    return var


@automation.register_action(
    "fader_buddy.run_self_calibration",
    RunSelfCalibrationAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(FaderBuddy),
    })
)
async def run_self_calibration_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    return var


@automation.register_action(
    "fader_buddy.update_firmware",
    UpdateFirmwareAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(FaderBuddy),
    })
)
async def update_firmware_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    return var
