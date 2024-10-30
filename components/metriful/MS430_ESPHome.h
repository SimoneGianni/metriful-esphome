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
#include "esphome/core/log.h"
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

    void set_temperature_sensor(Sensor *sensor) { this->temperature_s = sensor; }
    void set_pressure_sensor(Sensor *sensor) { this->pressure_s = sensor; }
    void set_humidity_sensor(Sensor *sensor) { this->humidity_s = sensor; }
    void set_particle_duty_sensor(Sensor *sensor) { this->particle_duty_s = sensor; }
    void set_particle_conc_sensor(Sensor *sensor) { this->particle_conc_s = sensor; }
    void set_gas_sensor(Sensor *sensor) { this->gas_s = sensor; }
    void set_aqi_sensor(Sensor *sensor) { this->aqi_s = sensor; }
    void set_co2e_sensor(Sensor *sensor) { this->CO2e_s = sensor; }
    void set_bvoc_sensor(Sensor *sensor) { this->bVOC_s = sensor; }
    void set_aqi_acc_sensor(Sensor *sensor) { this->aqi_acc_s = sensor; }
    void set_illuminance_sensor(Sensor *sensor) { this->illuminance_s = sensor; }
    void set_w_light_sensor(Sensor *sensor) { this->w_light_s = sensor; }
    void set_sound_spl_sensor(Sensor *sensor) { this->sound_spl_s = sensor; }
    void set_sound_peak_sensor(Sensor *sensor) { this->sound_peak_s = sensor; }

    // Sound band sensor setters for frequency bands
    void set_sound_band_sensor(uint8_t index, Sensor *sensor) {
      if (index < SOUND_FREQ_BANDS) {
        this->sound_bands_s[index] = sensor;
      }
    }

    void dump_config() {
      ESP_LOGCONFIG(TAG, "In dump config");
    }

    float get_setup_priority() const override {
      return esphome::setup_priority::BUS;
    }

    // Initialize the I2C bus and the MS430 board
    void setup() override {
      enableSerial = false;

      //SensorHardwareSetup(I2C_ADDRESS);
      /*
      uint8_t particleSensor = PARTICLE_SENSOR;
      TransmitI2C(I2C_ADDRESS, PARTICLE_SENSOR_SELECT_REG, &particleSensor, 1);
      uint8_t cyclePeriod = CYCLE_PERIOD;
      TransmitI2C(I2C_ADDRESS, CYCLE_TIME_PERIOD_REG, &cyclePeriod, 1);
      TransmitI2C(I2C_ADDRESS, CYCLE_MODE_CMD, 0, 0);
      */
    }

    void loop() override {
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