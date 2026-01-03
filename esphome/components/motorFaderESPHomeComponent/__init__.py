from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

MULTI_CONF = True
DEPENDENCIES = ["i2c"]

my_custom_sensor_ns = cg.esphome_ns.namespace("motorFaderESPHomeComponent")
MotorFaderESPHomeComponent = my_custom_sensor_ns.class_("MotorFaderESPHomeComponent", cg.PollingComponent, i2c.I2CDevice)

CONF_ON_MANUAL_MOVE = "on_manual_move"
CONF_ON_TOUCH_CHANGE = "on_touch_change"
CONF_ON_DOUBLE_TAP = "on_double_tap"
CONF_INVERT = "invert"
CONF_VALUE_CHANGE_RATE_LIMIT = "value_change_rate_limit"

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(MotorFaderESPHomeComponent),
        cv.Optional(CONF_ON_MANUAL_MOVE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_TOUCH_CHANGE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_DOUBLE_TAP): automation.validate_automation(single=True),
        cv.Optional(CONF_INVERT, default=False): cv.boolean,
        cv.Optional(CONF_VALUE_CHANGE_RATE_LIMIT): cv.positive_time_period_milliseconds,
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

    if CONF_VALUE_CHANGE_RATE_LIMIT in config:
        cg.add(var.set_value_change_rate_limit(config[CONF_VALUE_CHANGE_RATE_LIMIT]))

    if CONF_ON_MANUAL_MOVE in config:
        await automation.build_automation(
            var.get_on_manual_move_trigger(), [(cg.uint8, "x")], config[CONF_ON_MANUAL_MOVE]
        )

    if CONF_ON_TOUCH_CHANGE in config:
        await automation.build_automation(
            var.get_on_touch_change_trigger(), [(cg.bool_, "x")], config[CONF_ON_TOUCH_CHANGE]
        )

    if CONF_ON_DOUBLE_TAP in config:
        await automation.build_automation(
            var.get_on_double_tap_trigger(), [], config[CONF_ON_DOUBLE_TAP]
        )
