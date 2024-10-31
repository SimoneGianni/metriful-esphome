import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    UNIT_CELSIUS, UNIT_PERCENT, UNIT_OHM, UNIT_PARTS_PER_MILLION, UNIT_LUX, UNIT_PASCAL, 
    UNIT_DECIBEL, UNIT_MICROGRAMS_PER_CUBIC_METER, DEVICE_CLASS_TEMPERATURE, 
    DEVICE_CLASS_HUMIDITY, DEVICE_CLASS_PRESSURE, DEVICE_CLASS_SOUND_PRESSURE,
    DEVICE_CLASS_ILLUMINANCE, DEVICE_CLASS_AQI, DEVICE_CLASS_PM25, 
    STATE_CLASS_MEASUREMENT, CONF_ID
)
import logging

DEPENDENCIES = ["i2c"]

metriful_component_ns = cg.esphome_ns.namespace("metriful_ms430")
MetrifulComponent = metriful_component_ns.class_("MS430", cg.Component, i2c.I2CDevice)

SENSOR_SCHEMA = sensor.sensor_schema(
    accuracy_decimals=1, state_class=STATE_CLASS_MEASUREMENT
)

CONF_TEMPERATURE = "temperature"
CONF_PRESSURE = "pressure"
CONF_HUMIDITY = "humidity"
CONF_PARTICLE_DUTY = "particle_duty"
CONF_PARTICLE_CONC = "particle_conc"
CONF_GAS = "gas"
CONF_AQI = "aqi"
CONF_CO2E = "co2e"
CONF_BVOC = "bvoc"
CONF_AQI_ACC = "aqi_acc"
CONF_ILLUMINANCE = "illuminance"
CONF_W_LIGHT = "w_light"
CONF_SOUND_SPL = "sound_spl"
CONF_SOUND_PEAK = "sound_peak"
CONF_SOUND_BANDS = "sound_bands"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MetrifulComponent),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(),
        cv.Optional(CONF_PRESSURE): sensor.sensor_schema(),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(),
        cv.Optional(CONF_PARTICLE_DUTY): sensor.sensor_schema(),
        cv.Optional(CONF_PARTICLE_CONC): sensor.sensor_schema(),
        cv.Optional(CONF_GAS): sensor.sensor_schema(),
        cv.Optional(CONF_AQI): sensor.sensor_schema(),
        cv.Optional(CONF_CO2E): sensor.sensor_schema(),
        cv.Optional(CONF_BVOC): sensor.sensor_schema(),
        cv.Optional(CONF_AQI_ACC): sensor.sensor_schema(),
        cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(),
        cv.Optional(CONF_W_LIGHT): sensor.sensor_schema(),
        cv.Optional(CONF_SOUND_SPL): sensor.sensor_schema(),
        cv.Optional(CONF_SOUND_PEAK): sensor.sensor_schema(),
        cv.Optional(CONF_SOUND_BANDS): cv.All([sensor.sensor_schema()]),
    }
).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x71))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
