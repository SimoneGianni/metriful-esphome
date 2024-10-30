/*
  MS430_ESPHome.h

  This file creates an interface so that the MS430 Arduino code can
  be used as a custom sensor within ESPHome.

  Suitable for ESP8266, ESP32 and Raspberry Pi Pico W.

  Copyright 2023 Metriful Ltd.
  Licensed under the MIT License - for further details see LICENSE.txt

  For code examples, datasheet and user guide, visit
  https://github.com/metriful/sensor
*/

#include "esphome.h"
#include <stdint.h>
#include "Metriful_sensor.h"
#include "sensor_constants.h"


// Choose time interval for reading data (every 3, 100, or 300 seconds)
// 100 or 300 seconds are recommended to avoid self-heating.
#define CYCLE_PERIOD CYCLE_PERIOD_100_S

//////////////////////////////////////////////////////////////

extern bool enableSerial;

namespace esphome {
namespace metriful_ms430 {
using namespace sensor;
static const char *const TAG = "ms430";

class MS430 :  public i2c::I2CDevice, public Component
{
  public:
    Sensor * temperature_s = new Sensor();
    Sensor * pressure_s = new Sensor();
    Sensor * humidity_s = new Sensor();
    Sensor * particle_duty_s = new Sensor();
    Sensor * particle_conc_s = new Sensor();
    Sensor * gas_s = new Sensor();
    Sensor * aqi_s = new Sensor();
    Sensor * CO2e_s = new Sensor();
    Sensor * bVOC_s = new Sensor();
    Sensor * aqi_acc_s = new Sensor();
    Sensor * illuminance_s = new Sensor();
    Sensor * w_light_s = new Sensor();
    Sensor * sound_spl_s = new Sensor();
    Sensor * sound_peak_s = new Sensor();
    Sensor * sound_bands_s[SOUND_FREQ_BANDS] = {0};
    AirData_F_t airDataF = {0};
    AirQualityData_F_t airQualityDataF = {0};
    LightData_F_t lightDataF = {0};
    SoundData_F_t soundDataF = {0};
    ParticleData_F_t particleDataF = {0};
    bool firstOutput = true;
    bool firstAQIoutput = true;
    bool AQIinitialized = false;

    MS430()
    {
      for (uint8_t i = 0; i < SOUND_FREQ_BANDS; i++)
      {
        sound_bands_s[i] = new Sensor();
      }
    }

    void setupSensors() {
      ESP_LOGI(TAG, "setup sensors BEGIN");
      // Temperature sensor
      App.register_sensor(temperature_s);
      temperature_s->set_name("TEST Temperature");
      temperature_s->set_unit_of_measurement("°C");
      temperature_s->set_accuracy_decimals(1);
      temperature_s->set_icon("mdi:thermometer");
      temperature_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      temperature_s->set_device_class("temperature");

      // Air pressure sensor
      App.register_sensor(pressure_s);
      pressure_s->set_name("Air pressure");
      pressure_s->set_unit_of_measurement("Pa");
      pressure_s->set_accuracy_decimals(0);
      pressure_s->set_icon("mdi:weather-partly-rainy");
      pressure_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      pressure_s->set_device_class("atmospheric_pressure");

      // Humidity sensor
      App.register_sensor(humidity_s);
      humidity_s->set_name("Humidity");
      humidity_s->set_unit_of_measurement("%");
      humidity_s->set_accuracy_decimals(1);
      humidity_s->set_icon("mdi:cloud-percent");
      humidity_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      humidity_s->set_device_class("humidity");

      // Gas sensor
      App.register_sensor(gas_s);
      gas_s->set_name("Gas sensor resistance");
      gas_s->set_unit_of_measurement("Ω");
      gas_s->set_accuracy_decimals(0);
      gas_s->set_icon("mdi:scent");
      gas_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      gas_s->set_device_class("aqi");

      // Particle sensor duty cycle
      App.register_sensor(particle_duty_s);
      particle_duty_s->set_name("Particle sensor duty cycle");
      particle_duty_s->set_unit_of_measurement("%");
      particle_duty_s->set_accuracy_decimals(2);
      particle_duty_s->set_icon("mdi:square-wave");
      particle_duty_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      particle_duty_s->set_device_class("pm25");

      // Particle concentration
      App.register_sensor(particle_conc_s);
      particle_conc_s->set_name("Particle concentration");
      particle_conc_s->set_unit_of_measurement("μg/m³");
      particle_conc_s->set_accuracy_decimals(2);
      particle_conc_s->set_icon("mdi:chart-bubble");
      particle_conc_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      particle_conc_s->set_device_class("pm25");

      // Air quality index
      App.register_sensor(aqi_s);
      aqi_s->set_name("Air quality index");
      aqi_s->set_accuracy_decimals(1);
      aqi_s->set_icon("mdi:flower-tulip-outline");
      aqi_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      aqi_s->set_device_class("aqi");

      // Estimated CO2
      App.register_sensor(CO2e_s);
      CO2e_s->set_name("Estimated CO2");
      CO2e_s->set_unit_of_measurement("ppm");
      CO2e_s->set_accuracy_decimals(1);
      CO2e_s->set_icon("mdi:molecule-co2");
      CO2e_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      CO2e_s->set_device_class("carbon_dioxide");

      // Equivalent breath VOC
      App.register_sensor(bVOC_s);
      bVOC_s->set_name("Equivalent breath VOC");
      bVOC_s->set_unit_of_measurement("ppm");
      bVOC_s->set_accuracy_decimals(2);
      bVOC_s->set_icon("mdi:account-voice");
      bVOC_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      bVOC_s->set_device_class("volatile_organic_compounds_parts");

      // Air quality accuracy
      App.register_sensor(aqi_acc_s);
      aqi_acc_s->set_name("Air quality accuracy");
      aqi_acc_s->set_accuracy_decimals(0);
      aqi_acc_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);

      // Illuminance
      App.register_sensor(illuminance_s);
      illuminance_s->set_name("Illuminance");
      illuminance_s->set_unit_of_measurement("lux");
      illuminance_s->set_accuracy_decimals(2);
      illuminance_s->set_icon("mdi:white-balance-sunny");
      illuminance_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      illuminance_s->set_device_class("illuminance");

      // White light level
      App.register_sensor(w_light_s);
      w_light_s->set_name("White light level");
      w_light_s->set_accuracy_decimals(0);
      w_light_s->set_icon("mdi:circle-outline");
      w_light_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      w_light_s->set_device_class("illuminance");

      // Sound pressure level
      App.register_sensor(sound_spl_s);
      sound_spl_s->set_name("Sound pressure level");
      sound_spl_s->set_unit_of_measurement("dBA");
      sound_spl_s->set_accuracy_decimals(1);
      sound_spl_s->set_icon("mdi:microphone");
      sound_spl_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_spl_s->set_device_class("sound_pressure");

      // Peak sound amplitude
      App.register_sensor(sound_peak_s);
      sound_peak_s->set_name("Peak sound amplitude");
      sound_peak_s->set_unit_of_measurement("mPa");
      sound_peak_s->set_accuracy_decimals(2);
      sound_peak_s->set_icon("mdi:waveform");
      sound_peak_s->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_peak_s->set_device_class("sound_pressure");

      // Sound bands SPL sensors
      App.register_sensor(sound_bands_s[0]);
      sound_bands_s[0]->set_name("SPL at 125 Hz");
      sound_bands_s[0]->set_unit_of_measurement("dB");
      sound_bands_s[0]->set_accuracy_decimals(1);
      sound_bands_s[0]->set_icon("mdi:sine-wave");
      sound_bands_s[0]->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_bands_s[0]->set_device_class("sound_pressure");

      // SPL at 250 Hz
      App.register_sensor(sound_bands_s[1]);
      sound_bands_s[1]->set_name("SPL at 250 Hz");
      sound_bands_s[1]->set_unit_of_measurement("dB");
      sound_bands_s[1]->set_accuracy_decimals(1);
      sound_bands_s[1]->set_icon("mdi:sine-wave");
      sound_bands_s[1]->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_bands_s[1]->set_device_class("sound_pressure");

      // SPL at 500 Hz
      App.register_sensor(sound_bands_s[2]);
      sound_bands_s[2]->set_name("SPL at 500 Hz");
      sound_bands_s[2]->set_unit_of_measurement("dB");
      sound_bands_s[2]->set_accuracy_decimals(1);
      sound_bands_s[2]->set_icon("mdi:sine-wave");
      sound_bands_s[2]->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_bands_s[2]->set_device_class("sound_pressure");

      // SPL at 1000 Hz
      App.register_sensor(sound_bands_s[3]);
      sound_bands_s[3]->set_name("SPL at 1000 Hz");
      sound_bands_s[3]->set_unit_of_measurement("dB");
      sound_bands_s[3]->set_accuracy_decimals(1);
      sound_bands_s[3]->set_icon("mdi:sine-wave");
      sound_bands_s[3]->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_bands_s[3]->set_device_class("sound_pressure");

      // SPL at 2000 Hz
      App.register_sensor(sound_bands_s[4]);
      sound_bands_s[4]->set_name("SPL at 2000 Hz");
      sound_bands_s[4]->set_unit_of_measurement("dB");
      sound_bands_s[4]->set_accuracy_decimals(1);
      sound_bands_s[4]->set_icon("mdi:sine-wave");
      sound_bands_s[4]->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_bands_s[4]->set_device_class("sound_pressure");

      // SPL at 4000 Hz
      App.register_sensor(sound_bands_s[5]);
      sound_bands_s[5]->set_name("SPL at 4000 Hz");
      sound_bands_s[5]->set_unit_of_measurement("dB");
      sound_bands_s[5]->set_accuracy_decimals(1);
      sound_bands_s[5]->set_icon("mdi:sine-wave");
      sound_bands_s[5]->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
      sound_bands_s[5]->set_device_class("sound_pressure");
      
      ESP_LOGI(TAG, "setup sensors END");
    }

    float get_setup_priority() const override
    {
      return esphome::setup_priority::BUS;
    }

    // Initialize the I2C bus and the MS430 board
    void setup() override
    {
      enableSerial = false;
      setupSensors();
      //SensorHardwareSetup(I2C_ADDRESS);
      /*
      uint8_t particleSensor = PARTICLE_SENSOR;
      TransmitI2C(I2C_ADDRESS, PARTICLE_SENSOR_SELECT_REG, &particleSensor, 1);
      uint8_t cyclePeriod = CYCLE_PERIOD;
      TransmitI2C(I2C_ADDRESS, CYCLE_TIME_PERIOD_REG, &cyclePeriod, 1);
      TransmitI2C(I2C_ADDRESS, CYCLE_MODE_CMD, 0, 0);
      */
    }

    void loop() override
    {
      /*
      if (ready_assertion_event)
      {
        output();
        ready_assertion_event = false;
      }
      */
    }

    /*
    // Read data and send to Home Assistant
    void output()
    {
      airDataF = getAirDataF(I2C_ADDRESS);
      airQualityDataF = getAirQualityDataF(I2C_ADDRESS);
      lightDataF = getLightDataF(I2C_ADDRESS);
      soundDataF = getSoundDataF(I2C_ADDRESS);
      if (PARTICLE_SENSOR != PARTICLE_SENSOR_OFF)
      {
        particleDataF = getParticleDataF(I2C_ADDRESS);
        particle_duty_s->publish_state(particleDataF.duty_cycle_pc);
        particle_conc_s->publish_state(particleDataF.concentration);
      }
      temperature_s->publish_state(airDataF.T_C);
      pressure_s->publish_state(airDataF.P_Pa);
      humidity_s->publish_state(airDataF.H_pc);
      gas_s->publish_state(airDataF.G_Ohm);

      // Only publish air quality values when the algorithm has
      // initialized, and send initial dummy values to force update.
      if (firstOutput)
      {
        aqi_acc_s->publish_state(-1.0);
        firstOutput = false;
      }
      aqi_acc_s->publish_state(airQualityDataF.AQI_accuracy);
      if (airQualityDataF.AQI_accuracy > 0)
      {
        AQIinitialized = true;
      }
      if (AQIinitialized)
      {
        if (firstAQIoutput)
        {
          aqi_s->publish_state(-1.0);
          firstAQIoutput = false;
        }
        aqi_s->publish_state(airQualityDataF.AQI);
        CO2e_s->publish_state(airQualityDataF.CO2e);
        bVOC_s->publish_state(airQualityDataF.bVOC);
      }
      //
      illuminance_s->publish_state(lightDataF.illum_lux);
      w_light_s->publish_state(lightDataF.white);
      //
      sound_spl_s->publish_state(soundDataF.SPL_dBA);
      sound_peak_s->publish_state(soundDataF.peakAmp_mPa);
      for (uint8_t i = 0; i < SOUND_FREQ_BANDS; i++) {
        sound_bands_s[i]->publish_state(soundDataF.SPL_bands_dB[i]);
      }
    }
    */
};

} // namespace metriful_ms430
} // namespace esphome