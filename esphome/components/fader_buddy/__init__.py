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

import hashlib
import re
from pathlib import Path

import requests

from esphome import automation, external_files
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button, i2c

# Aliased: importing the sibling `text_sensor.py` platform module binds it as an
# attribute of this package, which would shadow a plain `text_sensor` name here.
from esphome.components import text_sensor as core_text_sensor
from esphome.const import CONF_ID, CONF_MODE, CONF_NAME
from esphome.core import CORE

MULTI_CONF = True
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["button", "text_sensor"]

DOMAIN = "fader_buddy"

# Flash geometry is read out of the component's own copy of bootloader_protocol.h
# rather than duplicated here, so there is one source of truth for it.
_BOOTLOADER_PROTOCOL_H = Path(__file__).parent / "bootloader_protocol.h"


def _bl_define(name: str) -> int:
    text = _BOOTLOADER_PROTOCOL_H.read_text()
    m = re.search(rf"#define\s+{re.escape(name)}\s+\(?\s*(0x[0-9A-Fa-f]+|\d+)", text)
    if m is None:
        raise cv.Invalid(f"Could not find {name} in {_BOOTLOADER_PROTOCOL_H}")
    return int(m.group(1), 0)


BL_FLASH_SIZE = _bl_define("BL_FLASH_SIZE")
BL_PAGE_SIZE = _bl_define("BL_PAGE_SIZE")
BL_APP_START = _bl_define("BL_BOOTEND") * 256

# export_app_image.py extracts from BL_APP_START through the FW_VERSION footer in the
# last 2 bytes of flash, so a correctly built image is always exactly this long --
# a cheap check that we were handed a real app image and not a truncated download,
# an Intel-hex, or some other file entirely.
EXPECTED_IMAGE_LENGTH = BL_FLASH_SIZE - BL_APP_START


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


# ---------------------------------------------------------------------------
# Released application images
#
# Images are published as assets on a GitHub release, tagged
# `releases/firmware/v<version>` -- so FW_VERSION 1.3 lives at tag
# `releases/firmware/v1.3`, and the asset URL can be determined from only a
# version.
#
# The sha256 is what makes fetching a remote binary safe: it pins the exact bytes,
# so a re-uploaded or substituted asset fails the build instead of being flashed
# onto a fader. Add an entry here when a release is tagged; the CI job that builds
# the release prints the line to paste (.github/workflows/firmware.yml).
# ---------------------------------------------------------------------------
GITHUB_REPO = "scottbez1/FaderBuddy"
RELEASE_TAG_PREFIX = "releases/firmware/"

KNOWN_FIRMWARE: dict[str, str] = {
    # "1.3": "<sha256 of fader_buddy_app_v1.3.bin>",
}


def _release_asset_name(version: str) -> str:
    return f"fader_buddy_app_v{version}.bin"


def _release_asset_url(version: str) -> str:
    return (
        f"https://github.com/{GITHUB_REPO}/releases/download/"
        f"{RELEASE_TAG_PREFIX}v{version}/{_release_asset_name(version)}"
    )


def _parse_version(value: str) -> tuple[int, int]:
    """'1.3' or 'v1.3' -> (1, 3). Matches FW_VERSION_MAJOR/MINOR in i2c_data.h."""
    m = re.fullmatch(r"v?(\d+)\.(\d+)", value.strip())
    if m is None:
        raise cv.Invalid(
            f"Firmware version '{value}' must be 'major.minor', e.g. '1.3' "
            "(matching FW_VERSION_MAJOR/FW_VERSION_MINOR in i2c_data.h)"
        )
    return int(m.group(1)), int(m.group(2))


def _normalize_version(value: str) -> str:
    major, minor = _parse_version(value)
    return f"{major}.{minor}"


def _download_release_image(url: str, sha256: str) -> bytes:
    """Fetch a release asset, verify its hash, and cache it by hash."""
    cache_path = external_files.compute_local_file_dir(DOMAIN) / f"{sha256}.bin"
    if cache_path.is_file():
        data = cache_path.read_bytes()
        if hashlib.sha256(data).hexdigest() == sha256:
            return data
        # A corrupted cache entry should not be sticky.
        cache_path.unlink()

    try:
        req = requests.get(url, timeout=external_files.NETWORK_TIMEOUT)
        req.raise_for_status()
    except requests.exceptions.RequestException as e:
        raise cv.Invalid(f"Could not download firmware image from {url}: {e}") from e

    data = req.content
    actual = hashlib.sha256(data).hexdigest()
    if actual != sha256:
        raise cv.Invalid(
            f"Firmware image from {url} has sha256 {actual}, expected {sha256}. "
            "Refusing to flash an image that is not the one this config pins."
        )
    cache_path.write_bytes(data)
    return data


# Cache of already-embedded firmware images, keyed by a source-identifying string, so
# multiple fader_buddy instances using the same image share one emitted array instead
# of duplicating a ~14 KB blob per instance (MULTI_CONF).
_firmware_image_cache: dict[str, tuple[str, int, int, int]] = {}


def _validate_image_bytes(data: bytes, source: str, expect_version: tuple[int, int] | None):
    if len(data) == 0 or len(data) % BL_PAGE_SIZE != 0:
        raise cv.Invalid(
            f"Firmware image '{source}' must be a non-empty multiple of {BL_PAGE_SIZE} bytes "
            "-- the exact page-aligned APPCODE image produced by "
            "firmware/tools/export_app_image.py, not an Intel-hex or .elf file."
        )
    if len(data) != EXPECTED_IMAGE_LENGTH:
        raise cv.Invalid(
            f"Firmware image '{source}' is {len(data)} bytes, expected {EXPECTED_IMAGE_LENGTH} "
            f"(flash {BL_FLASH_SIZE} - app start 0x{BL_APP_START:04X}). Produce it with "
            "firmware/tools/export_app_image.py."
        )

    # Last 2 bytes are the FW_VERSION_FOOTER (bootloader_protocol.h BL_APP_META_ADDR),
    # baked in by the firmware build itself -- always in sync with what the flashed app
    # will report via REG_FW_VERSION, no separate YAML field to keep in sync by hand.
    fw_version = (data[-2] << 8) | data[-1]
    if expect_version is not None:
        want = (expect_version[0] << 8) | expect_version[1]
        if fw_version != want:
            raise cv.Invalid(
                f"Firmware image '{source}' reports version "
                f"{fw_version >> 8}.{fw_version & 0xFF}, but the config asked for "
                f"{expect_version[0]}.{expect_version[1]}. The release asset is "
                "mislabelled or the wrong file was attached to the tag."
            )
    return fw_version


def _get_or_emit_firmware_image(
    data: bytes, cache_key: str, source: str, expect_version: tuple[int, int] | None = None
) -> tuple[str, int, int, int]:
    """Returns (symbol_name, length, crc16, fw_version), emitting the array once."""
    cached = _firmware_image_cache.get(cache_key)
    if cached is not None:
        return cached

    fw_version = _validate_image_bytes(data, source, expect_version)
    crc16 = _crc16_ccitt(data)

    symbol = f"fader_buddy_firmware_image_{len(_firmware_image_cache)}"
    array_body = ", ".join(str(b) for b in data)
    cg.add_global(
        cg.RawExpression(f"static const uint8_t {symbol}[{len(data)}] PROGMEM = {{{array_body}}}")
    )

    result = (symbol, len(data), crc16, fw_version)
    _firmware_image_cache[cache_key] = result
    return result


def _resolve_firmware(config) -> tuple[bytes, str, str, tuple[int, int] | None]:
    """-> (image bytes, cache key, human-readable source, expected version)."""
    version = config[CONF_VERSION]
    major, minor = _parse_version(version)
    url = config.get(CONF_URL) or _release_asset_url(version)
    sha256 = config.get(CONF_SHA256) or KNOWN_FIRMWARE.get(version)
    if sha256 is None:
        raise cv.Invalid(
            f"Firmware version {version} is not in KNOWN_FIRMWARE, so its sha256 is "
            f"unknown. Either add it to KNOWN_FIRMWARE in {__file__}, or give '{CONF_SHA256}' "
            "(and optionally 'url') explicitly in the config."
        )
    data = _download_release_image(url, sha256)
    return data, f"sha256:{sha256}", url, (major, minor)


def _validate_sha256(value):
    value = cv.string(value).lower()
    if not re.fullmatch(r"[0-9a-f]{64}", value):
        raise cv.Invalid("sha256 must be 64 hexadecimal characters")
    return value


# Resolution and image checks run as *validators*, not in to_code(), so a bad hash,
# an unreachable release or an image built for a different boot/app split is reported
# as a config error by `esphome config` rather than as a traceback partway through a
# compile. The download is cached by hash, so to_code() re-resolving is free.
def _validate_firmware(config):
    data, _cache_key, source, expect_version = _resolve_firmware(config)
    _validate_image_bytes(data, source, expect_version)
    return config


def _validate_firmware_image_file(value):
    value = cv.file_(value)
    path = Path(value).resolve()
    try:
        data = path.read_bytes()
    except OSError as e:
        raise cv.Invalid(f"Could not read firmware image '{value}': {e}") from e
    _validate_image_bytes(data, str(path), None)
    return value

fader_buddy_ns = cg.esphome_ns.namespace("fader_buddy")
FaderBuddy = fader_buddy_ns.class_("FaderBuddy", cg.PollingComponent, i2c.I2CDevice)
SelfCalibrationButton = fader_buddy_ns.class_(
    "SelfCalibrationButton", button.Button, cg.Parented.template(FaderBuddy)
)
FirmwareUpdateButton = fader_buddy_ns.class_(
    "FirmwareUpdateButton", button.Button, cg.Parented.template(FaderBuddy)
)

# Used by platform files (e.g. text_sensor) to reference the parent hub
CONF_FADER_BUDDY_ID = "fader_buddy_id"

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
CONF_FIRMWARE = "firmware"
CONF_VERSION = "version"
CONF_URL = "url"
CONF_SHA256 = "sha256"

# `firmware: "1.3"` is shorthand for `firmware: {version: "1.3"}`; url and sha256
# default from the release tag scheme and KNOWN_FIRMWARE respectively.
FIRMWARE_SCHEMA = cv.All(
    cv.maybe_simple_value(
        {
            cv.Required(CONF_VERSION): cv.string_strict,
            cv.Optional(CONF_URL): cv.url,
            cv.Optional(CONF_SHA256): _validate_sha256,
        },
        key=CONF_VERSION,
    ),
    _validate_firmware,
)
CONF_SPEED = "speed"
CONF_DEFAULT_SPEED = "default_speed"
CONF_SERIAL_NUMBER = "serial_number"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_SELF_CALIBRATION = "self_calibration"
CONF_FIRMWARE_UPDATE = "firmware_update"

# Diagnostic text sensors the hub creates itself, so a bare fader_buddy block
# reports what it is without any entity yaml. Each maps to (default name
# suffix, icon). Set `internal: true` on one to keep it out of Home Assistant,
# or `disabled_by_default: true` to have HA register it but leave it off.
AUTO_TEXT_SENSORS = {
    CONF_SERIAL_NUMBER: ("Serial Number", "mdi:identifier"),
    CONF_FIRMWARE_VERSION: ("Firmware Version", "mdi:chip"),
}

# Buttons the hub creates itself, same deal. A press here ties the fader up for
# seconds - sweeping the carriage, or rewriting its flash - so these are
# entity_category "config" rather than controls.
AUTO_BUTTONS = {
    CONF_SELF_CALIBRATION: ("Self Calibration", "mdi:tune-vertical"),
    CONF_FIRMWARE_UPDATE: ("Firmware Update", "mdi:package-down"),
}

AUTO_ENTITIES = {**AUTO_TEXT_SENSORS, **AUTO_BUTTONS}


def _default_entity_names(config):
    """Give each auto-created text sensor a name before its schema validates.

    The entity base schema rejects a sub-config carrying neither `name:` nor a
    manual `id:`, and it runs while validating that sub-config - before any
    validator on this schema could fill one in. So this has to be a
    pre-validator, seeing the raw config.

    The name is prefixed with the hub's id so several faders in one device don't
    collide. A config with multiple hubs and no ids on them would; ESPHome's
    auto-generated ids don't exist yet at this point.
    """
    if not isinstance(config, dict):
        return config
    hub_id = config.get(CONF_ID)
    prefix = f"{hub_id} " if isinstance(hub_id, str) else ""
    for key, (label, _icon) in AUTO_ENTITIES.items():
        sub = config.setdefault(key, {})
        if isinstance(sub, dict) and CONF_NAME not in sub:
            sub[CONF_NAME] = f"{prefix}{label}"
    return config


def _has_firmware(config):
    """True if this fader has a packaged image to install."""
    return CONF_FIRMWARE in config or CONF_FIRMWARE_IMAGE in config


def _claimed_by_legacy_platform(config, key):
    """True if a `text_sensor: platform: fader_buddy` block already provides this.

    The deprecated platform form calls the same setter, so without this check a
    config using it would get two entities for one sensor - the hub's own, left
    unpublished, and the platform's. Drop the hub's in that case, so migrating
    is a pure deletion of the old block.
    """
    for entry in CORE.config.get("text_sensor", []):
        if entry.get("platform") != "fader_buddy":
            continue
        if entry.get(CONF_FADER_BUDDY_ID) != config[CONF_ID]:
            continue
        if key in entry:
            return True
    return False

# Unitless move speed: 255 is full speed, 0 the slowest smooth motion
SPEED_FULL = 255

# Schema for a single layer haptic configuration
LAYER_HAPTIC_SCHEMA = cv.Schema({
    cv.Required(CONF_LAYER): cv.int_range(min=0, max=7),
    cv.Required(CONF_MODE): cv.enum(HAPTIC_MODES, lower=True),
    cv.Optional(CONF_DETENT_COUNT, default=0): cv.int_range(min=0, max=15),
    cv.Optional(CONF_DETENT_STRENGTH, default=0): cv.int_range(min=0, max=7),
    cv.Optional(CONF_VALUE_CHANGE_MIN_INTERVAL, default="0ms"): cv.positive_time_period_milliseconds,
    # Speed used for moves on this layer that don't name one. Kept host-side
    # and sent with each move rather than stored on the fader.
    cv.Optional(CONF_DEFAULT_SPEED, default=SPEED_FULL): cv.int_range(min=0, max=255),
})

CONFIG_SCHEMA = cv.All(
    _default_entity_names,
    cv.Schema({
        cv.GenerateID(): cv.declare_id(FaderBuddy),
        cv.Optional(CONF_SERIAL_NUMBER, default={}): core_text_sensor.text_sensor_schema(
            entity_category="diagnostic",
            icon=AUTO_TEXT_SENSORS[CONF_SERIAL_NUMBER][1],
        ),
        cv.Optional(CONF_FIRMWARE_VERSION, default={}): core_text_sensor.text_sensor_schema(
            entity_category="diagnostic",
            icon=AUTO_TEXT_SENSORS[CONF_FIRMWARE_VERSION][1],
        ),
        cv.Optional(CONF_SELF_CALIBRATION, default={}): button.button_schema(
            SelfCalibrationButton,
            entity_category="config",
            icon=AUTO_BUTTONS[CONF_SELF_CALIBRATION][1],
        ),
        # Only created when a firmware image is configured (see to_code); with
        # nothing to install the button would have nothing to do. Whether an
        # update is actually pending is runtime state, so a press still checks
        # that (see FaderBuddy::request_firmware_update).
        cv.Optional(CONF_FIRMWARE_UPDATE, default={}): button.button_schema(
            FirmwareUpdateButton,
            entity_category="config",
            icon=AUTO_BUTTONS[CONF_FIRMWARE_UPDATE][1],
        ),
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
        cv.Optional(CONF_FIRMWARE_IMAGE): _validate_firmware_image_file,
        # Released image fetched from its GitHub release asset and pinned by hash.
        # Mutually exclusive with firmware_image, which is the local-file escape
        # hatch for iterating on an unreleased build.
        cv.Optional(CONF_FIRMWARE): FIRMWARE_SCHEMA,
    })
    .extend(cv.polling_component_schema("50ms"))
    .extend(i2c.i2c_device_schema(0x20)),  # default I2C address
    cv.has_at_most_one_key(CONF_FIRMWARE, CONF_FIRMWARE_IMAGE),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_INVERT in config:
        cg.add(var.set_invert(config[CONF_INVERT]))

    if not _claimed_by_legacy_platform(config, CONF_SERIAL_NUMBER):
        sens = await core_text_sensor.new_text_sensor(config[CONF_SERIAL_NUMBER])
        cg.add(var.set_serial_text_sensor(sens))
    if not _claimed_by_legacy_platform(config, CONF_FIRMWARE_VERSION):
        sens = await core_text_sensor.new_text_sensor(config[CONF_FIRMWARE_VERSION])
        cg.add(var.set_firmware_text_sensor(sens))

    for key in AUTO_BUTTONS:
        # No firmware configured means nothing the button could ever install, so
        # don't put a dead entity in Home Assistant.
        if key == CONF_FIRMWARE_UPDATE and not _has_firmware(config):
            continue
        btn = await button.new_button(config[key])
        await cg.register_parented(btn, config[CONF_ID])

    # Store initial layer haptic configurations (sent during setup)
    if CONF_LAYER_HAPTICS in config:
        for haptic_config in config[CONF_LAYER_HAPTICS]:
            layer = haptic_config[CONF_LAYER]
            mode = haptic_config[CONF_MODE]
            detent_count = haptic_config[CONF_DETENT_COUNT]
            detent_strength = haptic_config[CONF_DETENT_STRENGTH]
            min_interval = haptic_config[CONF_VALUE_CHANGE_MIN_INTERVAL]
            default_speed = haptic_config[CONF_DEFAULT_SPEED]

            cg.add(var.store_initial_layer_haptic_config(
                layer, mode, detent_count, detent_strength
            ))
            cg.add(var.set_layer_value_change_min_interval(layer, min_interval))
            cg.add(var.set_layer_default_speed(layer, default_speed))

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

    image = None
    if CONF_FIRMWARE in config:
        data, cache_key, source, expect_version = _resolve_firmware(config[CONF_FIRMWARE])
        image = _get_or_emit_firmware_image(data, cache_key, source, expect_version)
    elif CONF_FIRMWARE_IMAGE in config:
        path = Path(config[CONF_FIRMWARE_IMAGE]).resolve()
        image = _get_or_emit_firmware_image(data=path.read_bytes(), cache_key=str(path), source=str(path))

    if image is not None:
        symbol, length, crc16, fw_version = image
        cg.add(var.set_firmware_image(cg.RawExpression(symbol), length, crc16, fw_version))


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
        # Unitless move speed, 0 (slowest smooth motion) to 255 (full speed).
        # This caps speed, it does not stretch the move to fill a duration, so
        # a shorter move takes proportionally less time. Omit it to use the
        # layer's default_speed.
        cv.Optional(CONF_SPEED): cv.templatable(cv.int_range(min=0, max=255)),
    })
)
async def remote_move_to_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    position = await cg.templatable(config[CONF_POSITION], args, cg.uint8)
    cg.add(var.set_position(position))
    layer = await cg.templatable(config[CONF_LAYER], args, cg.uint8)
    cg.add(var.set_layer(layer))
    if CONF_SPEED in config:
        speed = await cg.templatable(config[CONF_SPEED], args, cg.uint8)
        cg.add(var.set_speed(speed))
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
