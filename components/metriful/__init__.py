import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    UNIT_CELSIUS, UNIT_PERCENT, UNIT_OHM, UNIT_PARTS_PER_MILLION, UNIT_LUX, UNIT_PA, UNIT_DBA, 
    UNIT_DBC, UNIT_MICROGRAM_PER_CUBIC_METER, DEVICE_CLASS_TEMPERATURE, 
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
        cv.Optional(CONF_TEMPERATURE, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_CELSIUS).accuracy_decimals(1).device_class(DEVICE_CLASS_TEMPERATURE).state_class(STATE_CLASS_MEASUREMENT)): sensor.sensor_schema(),
        cv.Optional(CONF_PRESSURE, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_PA).accuracy_decimals(0).device_class(DEVICE_CLASS_PRESSURE)): sensor.sensor_schema(),
        cv.Optional(CONF_HUMIDITY, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_PERCENT).accuracy_decimals(1).device_class(DEVICE_CLASS_HUMIDITY)): sensor.sensor_schema(),
        cv.Optional(CONF_PARTICLE_DUTY, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_PERCENT).accuracy_decimals(2).device_class(DEVICE_CLASS_PM25)): sensor.sensor_schema(),
        cv.Optional(CONF_PARTICLE_CONC, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_MICROGRAM_PER_CUBIC_METER).accuracy_decimals(2).device_class(DEVICE_CLASS_PM25)): sensor.sensor_schema(),
        cv.Optional(CONF_GAS, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_OHM).accuracy_decimals(0).device_class(DEVICE_CLASS_AQI)): sensor.sensor_schema(),
        cv.Optional(CONF_AQI, default=SENSOR_SCHEMA.accuracy_decimals(1).device_class(DEVICE_CLASS_AQI)): sensor.sensor_schema(),
        cv.Optional(CONF_CO2E, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_PARTS_PER_MILLION).accuracy_decimals(1).device_class(DEVICE_CLASS_AQI)): sensor.sensor_schema(),
        cv.Optional(CONF_BVOC, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_PARTS_PER_MILLION).accuracy_decimals(2).device_class(DEVICE_CLASS_AQI)): sensor.sensor_schema(),
        cv.Optional(CONF_AQI_ACC, default=SENSOR_SCHEMA.accuracy_decimals(0).device_class(DEVICE_CLASS_AQI)): sensor.sensor_schema(),
        cv.Optional(CONF_ILLUMINANCE, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_LUX).accuracy_decimals(2).device_class(DEVICE_CLASS_ILLUMINANCE)): sensor.sensor_schema(),
        cv.Optional(CONF_W_LIGHT, default=SENSOR_SCHEMA.accuracy_decimals(0).device_class(DEVICE_CLASS_ILLUMINANCE)): sensor.sensor_schema(),
        cv.Optional(CONF_SOUND_SPL, default=SENSOR_SCHEMA.unit_of_measurement(UNIT_DBA).accuracy_decimals(1).device_class(DEVICE_CLASS_SOUND_PRESSURE)): sensor.sensor_schema(),
        cv.Optional(CONF_SOUND_PEAK, default=SENSOR_SCHEMA.unit_of_measurement("mPa").accuracy_decimals(2).device_class(DEVICE_CLASS_SOUND_PRESSURE)): sensor.sensor_schema(),
        cv.Optional(CONF_SOUND_BANDS, default=[SENSOR_SCHEMA.unit_of_measurement(UNIT_DBC).accuracy_decimals(1).device_class(DEVICE_CLASS_SOUND_PRESSURE)] * 6): cv.All([sensor.sensor_schema()], cv.ensure_list_length(6)),
    }
).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x01))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    for key in [CONF_TEMPERATURE, CONF_PRESSURE, CONF_HUMIDITY, CONF_PARTICLE_DUTY, CONF_PARTICLE_CONC, CONF_GAS, CONF_AQI, CONF_CO2E, CONF_BVOC, CONF_AQI_ACC, CONF_ILLUMINANCE, CONF_W_LIGHT, CONF_SOUND_SPL, CONF_SOUND_PEAK]:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f'set_{key}_sensor')(sens))

    for i, band_sensor in enumerate(config[CONF_SOUND_BANDS]):
        sens = await sensor.new_sensor(band_sensor)
        cg.add(var.set_sound_band_sensor(i, sens))