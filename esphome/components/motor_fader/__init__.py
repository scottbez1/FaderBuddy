from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_MODE

MULTI_CONF = True
DEPENDENCIES = ["i2c"]

motor_fader_ns = cg.esphome_ns.namespace("motor_fader")
MotorFader = motor_fader_ns.class_("MotorFader", cg.PollingComponent, i2c.I2CDevice)

# Define haptic mode enum (in global namespace, shared with firmware)
HapticMode = cg.global_ns.enum("HapticMode")
HAPTIC_MODES = {
    "smooth": HapticMode.HAPTIC_NO_HAPTICS,
    "smooth_with_magnets": HapticMode.HAPTIC_SMOOTH_WITH_MAGNET_ENDS,
    "detents": HapticMode.HAPTIC_DETENTS,
}

CONF_ON_MANUAL_MOVE = "on_manual_move"
CONF_ON_TOUCH_CHANGE = "on_touch_change"
CONF_ON_DOUBLE_TAP = "on_double_tap"
CONF_INVERT = "invert"
CONF_LAYER_HAPTICS = "layer_haptics"
CONF_LAYER = "layer"
CONF_DETENT_COUNT = "detent_count"
CONF_DETENT_STRENGTH = "detent_strength"
CONF_POSITION = "position"
CONF_VALUE_CHANGE_MIN_INTERVAL = "value_change_min_interval"

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
        cv.GenerateID(): cv.declare_id(MotorFader),
        cv.Optional(CONF_ON_MANUAL_MOVE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_TOUCH_CHANGE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_DOUBLE_TAP): automation.validate_automation(single=True),
        cv.Optional(CONF_INVERT, default=False): cv.boolean,
        cv.Optional(CONF_LAYER_HAPTICS): cv.ensure_list(LAYER_HAPTIC_SCHEMA),
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

    if CONF_ON_TOUCH_CHANGE in config:
        await automation.build_automation(
            var.get_on_touch_change_trigger(), [(cg.bool_, "x"), (cg.uint8, "layer")], config[CONF_ON_TOUCH_CHANGE]
        )

    if CONF_ON_DOUBLE_TAP in config:
        await automation.build_automation(
            var.get_on_double_tap_trigger(), [(cg.uint8, "layer")], config[CONF_ON_DOUBLE_TAP]
        )


# Actions
SetActiveLayerAction = motor_fader_ns.class_("SetActiveLayerAction", automation.Action)
RemoteMoveToAction = motor_fader_ns.class_("RemoteMoveToAction", automation.Action)
SetLayerHapticConfigAction = motor_fader_ns.class_("SetLayerHapticConfigAction", automation.Action)
RunSelfCalibrationAction = motor_fader_ns.class_("RunSelfCalibrationAction", automation.Action)


@automation.register_action(
    "motor_fader.set_active_layer",
    SetActiveLayerAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(MotorFader),
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
    "motor_fader.remote_move_to",
    RemoteMoveToAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(MotorFader),
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
    "motor_fader.set_layer_haptic_config",
    SetLayerHapticConfigAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(MotorFader),
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
    "motor_fader.run_self_calibration",
    RunSelfCalibrationAction,
    cv.Schema({
        cv.Required(CONF_ID): cv.use_id(MotorFader),
    })
)
async def run_self_calibration_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    return var
