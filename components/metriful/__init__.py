import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID
import logging

DEPENDENCIES = ["i2c"]

metriful_component_ns = cg.esphome_ns.namespace("metriful_ms430")
MetrifulComponent = metriful_component_ns.class_("MS430", cg.Component, i2c.I2CDevice)


CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(MetrifulComponent)})
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x01))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)