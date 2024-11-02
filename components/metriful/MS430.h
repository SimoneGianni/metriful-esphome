/*
  Licensed under the MIT License - for further details see LICENSE.txt

  For code examples, datasheet and user guide, visit
  https://github.com/SimoneGianni/metriful-esphome
  https://github.com/metriful/sensor
*/

#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include <stdint.h>
#include "sensor_constants.h"

namespace esphome {
namespace metriful_ms430 {

using namespace sensor;

class MS430 : public i2c::I2CDevice, public Component {
public:
  // Constructor
  MS430();

  // Inline simple setters
  void set_temperature_sensor(Sensor *sensor) { temperature_s = sensor; }
  void set_pressure_sensor(Sensor *sensor) { pressure_s = sensor; }
  void set_humidity_sensor(Sensor *sensor) { humidity_s = sensor; }
  void set_particle_duty_sensor(Sensor *sensor) { particle_duty_s = sensor; }
  void set_particle_conc_sensor(Sensor *sensor) { particle_conc_s = sensor; }
  void set_gas_sensor(Sensor *sensor) { gas_s = sensor; }
  void set_aqi_sensor(Sensor *sensor) { aqi_s = sensor; }
  void set_co2e_sensor(Sensor *sensor) { CO2e_s = sensor; }
  void set_bvoc_sensor(Sensor *sensor) { bVOC_s = sensor; }
  void set_aqi_acc_sensor(Sensor *sensor) { aqi_acc_s = sensor; }
  void set_illuminance_sensor(Sensor *sensor) { illuminance_s = sensor; }
  void set_w_light_sensor(Sensor *sensor) { w_light_s = sensor; }
  void set_sound_spl_sensor(Sensor *sensor) { sound_spl_s = sensor; }
  void set_sound_peak_sensor(Sensor *sensor) { sound_peak_s = sensor; }
  void set_spl_125hz_sensor(Sensor *sensor) { sound_bands_s[0] = sensor; }
  void set_spl_250hz_sensor(Sensor *sensor) { sound_bands_s[1] = sensor; }
  void set_spl_500hz_sensor(Sensor *sensor) { sound_bands_s[2] = sensor; }
  void set_spl_1000hz_sensor(Sensor *sensor) { sound_bands_s[3] = sensor; }
  void set_spl_2000hz_sensor(Sensor *sensor) { sound_bands_s[4] = sensor; }
  void set_spl_4000hz_sensor(Sensor *sensor) { sound_bands_s[5] = sensor; }

  void set_ready_pin(int pin) { ready_pin = pin; }
  void set_cycle_time(int cycle_time) { this->cycle_time = cycle_time; cycle_time_changed = true; }

  // Override functions
  void dump_config() override;
  float get_setup_priority() const override;
  void setup() override;
  void loop() override;

  // I2C communication
  bool transmitI2C(uint8_t commandRegister, const uint8_t * data, uint8_t data_length);
  bool receiveI2C(uint8_t commandRegister, uint8_t data[], uint8_t data_length);

private:
  void readSensors();
  SoundData_t getSoundData();
  AirData_t getAirData();
  LightData_t getLightData();
  AirQualityData_t getAirQualityData();
  ParticleData_t getParticleData();
  SoundData_F_t getSoundDataF();
  AirData_F_t getAirDataF();
  LightData_F_t getLightDataF();
  AirQualityData_F_t getAirQualityDataF();
  ParticleData_F_t getParticleDataF();

  float convertEncodedTemperatureToFloat(uint8_t T_C_int_with_sign, uint8_t T_C_fr_1dp);
  void convertAirDataF(const AirData_t * airData_in, AirData_F_t * airDataF_out);
  void convertAirQualityDataF(const AirQualityData_t * airQualityData_in, AirQualityData_F_t * airQualityDataF_out);
  void convertLightDataF(const LightData_t * lightData_in, LightData_F_t * lightDataF_out);
  void convertSoundDataF(const SoundData_t * soundData_in, SoundData_F_t * soundDataF_out);
  void convertParticleDataF(const ParticleData_t * particleData_in, ParticleData_F_t * particleDataF_out);

  // Sensor pointers with direct initializations
  Sensor *temperature_s = new Sensor();
  Sensor *pressure_s = new Sensor();
  Sensor *humidity_s = new Sensor();
  Sensor *particle_duty_s = new Sensor();
  Sensor *particle_conc_s = new Sensor();
  Sensor *gas_s = new Sensor();
  Sensor *aqi_s = new Sensor();
  Sensor *CO2e_s = new Sensor();
  Sensor *bVOC_s = new Sensor();
  Sensor *aqi_acc_s = new Sensor();
  Sensor *illuminance_s = new Sensor();
  Sensor *w_light_s = new Sensor();
  Sensor *sound_spl_s = new Sensor();
  Sensor *sound_peak_s = new Sensor();
  Sensor *sound_bands_s[SOUND_FREQ_BANDS] = {nullptr};

  // Data structures with zero-initialization
  AirData_F_t airDataF = {0};
  AirQualityData_F_t airQualityDataF = {0};
  LightData_F_t lightDataF = {0};
  SoundData_F_t soundDataF = {0};
  ParticleData_F_t particleDataF = {0};

  // Member variables with default values
  bool firstOutput = true;
  bool firstAQIoutput = true;
  bool AQIinitialized = false;
  int ready_pin = 0;
  int cycle_time = 0;
  bool cycle_time_changed = true;
  int comm_state = 0;
  long last_report = 0;
};

} // namespace metriful_ms430
} // namespace esphome
