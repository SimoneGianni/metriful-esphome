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
CONF_SPL_125HZ = "spl_125hz"
CONF_SPL_250HZ = "spl_250hz"
CONF_SPL_500HZ = "spl_500hz"
CONF_SPL_1000HZ = "spl_1000hz"
CONF_SPL_2000HZ = "spl_2000hz"
CONF_SPL_4000HZ = "spl_4000hz"

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
        cv.Optional(CONF_SPL_125HZ): sensor.sensor_schema(),
        cv.Optional(CONF_SPL_250HZ): sensor.sensor_schema(),
        cv.Optional(CONF_SPL_500HZ): sensor.sensor_schema(),
        cv.Optional(CONF_SPL_1000HZ): sensor.sensor_schema(),
        cv.Optional(CONF_SPL_2000HZ): sensor.sensor_schema(),
        cv.Optional(CONF_SPL_4000HZ): sensor.sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x71))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Set up sensor configuration, falling back to defaults if not defined in config
    async def configure_sensor(key, name, unit, accuracy_decimals, icon, device_class):
        if key in config:
            sens = await sensor.new_sensor(config[key])
        else:
            sens = await sensor.new_sensor({
                "id": {
                    value: key,
                    type: "Sensor"
                },
                "name": name,
                "unit_of_measurement": unit,
                "accuracy_decimals": accuracy_decimals,
                "icon": icon,
                "device_class": device_class,
                "state_class": STATE_CLASS_MEASUREMENT,
            })
        cg.add(getattr(var, f'set_{key}_sensor')(sens))

    # Configure each sensor with the provided labels, units, and icons
    await configure_sensor(CONF_TEMPERATURE, "Temperature", UNIT_CELSIUS, 1, "mdi:thermometer", DEVICE_CLASS_TEMPERATURE)
    await configure_sensor(CONF_PRESSURE, "Air pressure", UNIT_PASCAL, 0, "mdi:weather-partly-rainy", DEVICE_CLASS_PRESSURE)
    await configure_sensor(CONF_HUMIDITY, "Humidity", UNIT_PERCENT, 1, "mdi:cloud-percent", DEVICE_CLASS_HUMIDITY)
    await configure_sensor(CONF_PARTICLE_DUTY, "Particle sensor duty cycle", UNIT_PERCENT, 2, "mdi:square-wave", DEVICE_CLASS_PM25)
    await configure_sensor(CONF_PARTICLE_CONC, "Particle concentration", UNIT_MICROGRAMS_PER_CUBIC_METER, 2, "mdi:chart-bubble", DEVICE_CLASS_PM25)
    await configure_sensor(CONF_GAS, "Gas sensor resistance", UNIT_OHM, 0, "mdi:scent", DEVICE_CLASS_AQI)
    await configure_sensor(CONF_AQI, "Air quality index", None, 1, "mdi:flower-tulip-outline", DEVICE_CLASS_AQI)
    await configure_sensor(CONF_CO2E, "Estimated CO₂", UNIT_PARTS_PER_MILLION, 1, "mdi:molecule-co2", "carbon_dioxide")
    await configure_sensor(CONF_BVOC, "Equivalent breath VOC", UNIT_PARTS_PER_MILLION, 2, "mdi:account-voice", "volatile_organic_compounds_parts")
    await configure_sensor(CONF_AQI_ACC, "Air quality accuracy", None, 0, None, DEVICE_CLASS_AQI)
    await configure_sensor(CONF_ILLUMINANCE, "Illuminance", UNIT_LUX, 2, "mdi:white-balance-sunny", DEVICE_CLASS_ILLUMINANCE)
    await configure_sensor(CONF_W_LIGHT, "White light level", None, 0, "mdi:circle-outline", DEVICE_CLASS_ILLUMINANCE)
    await configure_sensor(CONF_SOUND_SPL, "Sound pressure level", "dBA", 1, "mdi:microphone", DEVICE_CLASS_SOUND_PRESSURE)
    await configure_sensor(CONF_SOUND_PEAK, "Peak sound amplitude", "mPa", 2, "mdi:waveform", DEVICE_CLASS_SOUND_PRESSURE)
    
    # Configure sound bands without array structure
    await configure_sensor(CONF_SPL_125HZ, "SPL at 125 Hz", UNIT_DECIBEL, 1, "mdi:sine-wave", DEVICE_CLASS_SOUND_PRESSURE)
    await configure_sensor(CONF_SPL_250HZ, "SPL at 250 Hz", UNIT_DECIBEL, 1, "mdi:sine-wave", DEVICE_CLASS_SOUND_PRESSURE)
    await configure_sensor(CONF_SPL_500HZ, "SPL at 500 Hz", UNIT_DECIBEL, 1, "mdi:sine-wave", DEVICE_CLASS_SOUND_PRESSURE)
    await configure_sensor(CONF_SPL_1000HZ, "SPL at 1000 Hz", UNIT_DECIBEL, 1, "mdi:sine-wave", DEVICE_CLASS_SOUND_PRESSURE)
    await configure_sensor(CONF_SPL_2000HZ, "SPL at 2000 Hz", UNIT_DECIBEL, 1, "mdi:sine-wave", DEVICE_CLASS_SOUND_PRESSURE)
    await configure_sensor(CONF_SPL_4000HZ, "SPL at 4000 Hz", UNIT_DECIBEL, 1, "mdi:sine-wave", DEVICE_CLASS_SOUND_PRESSURE)