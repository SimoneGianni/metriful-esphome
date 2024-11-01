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

volatile bool ready_assertion_event = true;

// This function is automatically called after a falling edge
// (assertion) of READY and the flag variable is set true - it
// must be set false again in the main program.
void ISR_ATTRIBUTE ready_ISR(void)
{
  ready_assertion_event = true;
}

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

    int ready_pin = 0;
    int cycle_time = 0;
    bool cycle_time_changed = true;

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

    // Separate setters for each sound frequency band
    void set_spl_125hz_sensor(Sensor *sensor) {this->sound_bands_s[0] = sensor; };
    void set_spl_250hz_sensor(Sensor *sensor) {this->sound_bands_s[1] = sensor; };
    void set_spl_500hz_sensor(Sensor *sensor) {this->sound_bands_s[2] = sensor; };
    void set_spl_1000hz_sensor(Sensor *sensor) {this->sound_bands_s[3] = sensor; };
    void set_spl_2000hz_sensor(Sensor *sensor) {this->sound_bands_s[4] = sensor; };
    void set_spl_4000hz_sensor(Sensor *sensor) {this->sound_bands_s[5] = sensor; };

    void set_ready_pin(int pin) { this->ready_pin = pin; };
    void set_cycle_time(int cycle_time) { 
      this->cycle_time = cycle_time; 
      this->cycle_time_changed = true;
    };

    void dump_config() {
      ESP_LOGCONFIG(TAG, "In dump config");
      LOG_SENSOR("  ", "temperature_s", this->temperature_s);
      LOG_SENSOR("  ", "pressure_s", this->pressure_s);
      LOG_SENSOR("  ", "humidity_s", this->humidity_s);
      LOG_SENSOR("  ", "particle_duty_s", this->particle_duty_s);
      LOG_SENSOR("  ", "particle_conc_s", this->particle_conc_s);
      LOG_SENSOR("  ", "gas_s", this->gas_s);
      LOG_SENSOR("  ", "aqi_s", this->aqi_s);
      LOG_SENSOR("  ", "CO2e_s", this->CO2e_s);
      LOG_SENSOR("  ", "bVOC_s", this->bVOC_s);
      LOG_SENSOR("  ", "aqi_acc_s", this->aqi_acc_s);
      LOG_SENSOR("  ", "illuminance_s", this->illuminance_s);
      LOG_SENSOR("  ", "w_light_s", this->w_light_s);
      LOG_SENSOR("  ", "sound_spl_s", this->sound_spl_s);
      LOG_SENSOR("  ", "sound_peak_s", this->sound_peak_s);
    }

    float get_setup_priority() const override {
      return esphome::setup_priority::BUS;
    }

    ////////////////////////////////////////////////////////////////////////

    // Send data to the Metriful MS430 using the I2C ("two wire") interface.
    //
    // Returns true on success, false on failure.
    //
    // commandRegister = the settings register/command code to be used.
    // data = array containing the data to be sent.
    // data_length = the number of bytes from the "data" array to be sent.
    //
    bool transmitI2C(uint8_t commandRegister, const uint8_t * data, uint8_t data_length)
    {
      i2c::WriteBuffer buffers[2];
      buffers[0].data = &commandRegister;
      buffers[0].len = 1;
      buffers[1].data = data;
      buffers[1].len = data_length;
      return (bus_->writev(address_, buffers, 2, true) == i2c::ERROR_OK);
    }

    // Read data from the Metriful MS430 using the I2C ("two wire") interface.
    //
    // Returns true on success, false on failure.
    //
    // commandRegister = the settings register/data code to be used.
    // data = array to store the received data.
    // data_length = the number of bytes to read. 
    //
    bool receiveI2C(uint8_t commandRegister,
                    uint8_t data[], uint8_t data_length)
    {
      i2c::WriteBuffer buffers[1];
      buffers[0].data = &commandRegister;
      buffers[0].len = 1;
      if (bus_->writev(address_, buffers, 1, false) != i2c::ERROR_OK)
      {
        return false;
      }

      this->read(data, data_length);
      return true;
    }


    // Initialize the I2C bus and the MS430 board
    void setup() override {
      ESP_LOGCONFIG(TAG, "Setting up MS430");

      pinMode(this->ready_pin, INPUT);
      attachInterrupt(digitalPinToInterrupt(this->ready_pin), ready_ISR, FALLING);



      /* TODO enable particle sensor setting
      uint8_t particleSensor = PARTICLE_SENSOR;
      TransmitI2C(I2C_ADDRESS, PARTICLE_SENSOR_SELECT_REG, &particleSensor, 1);
      */
    }

    void loop() override {
      if (ready_assertion_event)
      {
        ESP_LOGV(TAG, "Got assertion event");
        //if (this->cycle_time_changed)
        //{
          ESP_LOGV(TAG, "Updating cycle time");
          this->cycle_time_changed = false;
          this->transmitI2C(RESET_CMD, 0, 0);
          uint8_t cyclePeriod = this->cycle_time;
          if (cyclePeriod >= 300) {
            cyclePeriod = 2;
          } else if (cyclePeriod >= 100) {
            cyclePeriod = 1;
          } else {
            cyclePeriod = 0;
          }
          this->transmitI2C(CYCLE_TIME_PERIOD_REG, &cyclePeriod, 1);
          this->transmitI2C(CYCLE_MODE_CMD, 0, 0);
        }
        // output();
        ready_assertion_event = false;
      }
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


    ////////////////////////////////////////////////////////////////////////

    // Convenience functions for reading data (integer representation)
    //
    // For each category of data (air, sound, etc.) a pointer to the data 
    // struct is passed to the ReceiveI2C() function. The received byte 
    // sequence fills the data struct in the correct order so that each 
    // field within the struct receives the value of an environmental data
    // quantity (temperature, sound level, etc.) 

    SoundData_t getSoundData()
    {
      SoundData_t soundData = {0};
      this->receiveI2C(SOUND_DATA_READ, (uint8_t *) &soundData, SOUND_DATA_BYTES);
      return soundData;
    }

    AirData_t getAirData()
    {
      AirData_t airData = {0};
      this->receiveI2C(AIR_DATA_READ, (uint8_t *) &airData, AIR_DATA_BYTES);
      return airData;
    }

    LightData_t getLightData()
    {
      LightData_t lightData = {0};
      this->receiveI2C(LIGHT_DATA_READ, (uint8_t *) &lightData, LIGHT_DATA_BYTES);
      return lightData;
    }

    AirQualityData_t getAirQualityData()
    {
      AirQualityData_t airQualityData = {0};
      this->receiveI2C(AIR_QUALITY_DATA_READ, (uint8_t *) &airQualityData, AIR_QUALITY_DATA_BYTES);
      return airQualityData;
    }

    ParticleData_t getParticleData()
    {
      ParticleData_t particleData = {0};
      this->receiveI2C(PARTICLE_DATA_READ, (uint8_t *) &particleData, PARTICLE_DATA_BYTES);
      return particleData;
    }

    // Convenience functions for reading data (float representation)

    SoundData_F_t getSoundDataF()
    {
      SoundData_F_t soundDataF = {0};
      SoundData_t soundData = this->getSoundData();
      this->convertSoundDataF(&soundData, &soundDataF);
      return soundDataF;
    }

    AirData_F_t getAirDataF()
    {
      AirData_F_t airDataF = {0};
      AirData_t airData = this->getAirData();
      this->convertAirDataF(&airData, &airDataF);
      return airDataF;
    }

    LightData_F_t getLightDataF()
    {
      LightData_F_t lightDataF = {0};
      LightData_t lightData = this->getLightData();
      this->convertLightDataF(&lightData, &lightDataF);
      return lightDataF;
    }

    AirQualityData_F_t getAirQualityDataF()
    {
      AirQualityData_F_t airQualityDataF = {0};
      AirQualityData_t airQualityData = this->getAirQualityData();
      this->convertAirQualityDataF(&airQualityData, &airQualityDataF);
      return airQualityDataF;
    }

    ParticleData_F_t getParticleDataF()
    {
      ParticleData_F_t particleDataF = {0};
      ParticleData_t particleData = this->getParticleData();
      this->convertParticleDataF(&particleData, &particleDataF);
      return particleDataF;
    }


    ////////////////////////////////////////////////////////////////////////

    // Functions to convert data from integer representation to floating-point
    // representation. Floats are easy to use for writing programs but require
    // greater memory and processing power resources, so may not always be
    // appropriate.

    // Decode and convert the temperature as read from the MS430 (integer
    // representation) into a float value 
    float convertEncodedTemperatureToFloat(uint8_t T_C_int_with_sign,
                                          uint8_t T_C_fr_1dp)
    {
      float temperature_C = ((float) (T_C_int_with_sign & TEMPERATURE_VALUE_MASK))
                              + (((float) T_C_fr_1dp) / 10.0f);
      if ((T_C_int_with_sign & TEMPERATURE_SIGN_MASK) != 0)
      {
        // The most-significant bit is set, which indicates that
        // the temperature is negative
        temperature_C = -temperature_C;
      }
      return temperature_C;
    }


    void convertAirDataF(const AirData_t * airData_in, AirData_F_t * airDataF_out)
    {
      // Decode the signed value for T (in Celsius)
      airDataF_out->T_C = convertEncodedTemperatureToFloat(
                            airData_in->T_C_int_with_sign,
                            airData_in->T_C_fr_1dp);
      airDataF_out->P_Pa = airData_in->P_Pa;
      airDataF_out->H_pc = ((float) airData_in->H_pc_int)
                            + (((float) airData_in->H_pc_fr_1dp) / 10.0f);
      airDataF_out->G_Ohm = airData_in->G_ohm;
    }

    void convertAirQualityDataF(const AirQualityData_t * airQualityData_in, 
                                    AirQualityData_F_t * airQualityDataF_out)
    {
      airQualityDataF_out->AQI =  ((float) airQualityData_in->AQI_int) + 
                                (((float) airQualityData_in->AQI_fr_1dp) / 10.0f);
      airQualityDataF_out->CO2e = ((float) airQualityData_in->CO2e_int) + 
                                (((float) airQualityData_in->CO2e_fr_1dp) / 10.0f);
      airQualityDataF_out->bVOC = ((float) airQualityData_in->bVOC_int) + 
                                (((float) airQualityData_in->bVOC_fr_2dp) / 100.0f);
      airQualityDataF_out->AQI_accuracy = airQualityData_in->AQI_accuracy;
    }

    void convertLightDataF(const LightData_t * lightData_in,
                          LightData_F_t * lightDataF_out)
    {
      lightDataF_out->illum_lux = ((float) lightData_in->illum_lux_int)
                              + (((float) lightData_in->illum_lux_fr_2dp) / 100.0f);
      lightDataF_out->white = lightData_in->white;
    }

    void convertSoundDataF(const SoundData_t * soundData_in,
                          SoundData_F_t * soundDataF_out)
    {
      soundDataF_out->SPL_dBA = ((float) soundData_in->SPL_dBA_int)
                          + (((float) soundData_in->SPL_dBA_fr_1dp) / 10.0f);
      for (uint16_t i = 0; i < SOUND_FREQ_BANDS; i++)
      {
        soundDataF_out->SPL_bands_dB[i] = ((float) soundData_in->SPL_bands_dB_int[i])
                          + (((float) soundData_in->SPL_bands_dB_fr_1dp[i]) / 10.0f);
      }
      soundDataF_out->peakAmp_mPa = ((float) soundData_in->peak_amp_mPa_int)
                          + (((float) soundData_in->peak_amp_mPa_fr_2dp) / 100.0f);
      soundDataF_out->stable = (soundData_in->stable == 1);
    }

    void convertParticleDataF(const ParticleData_t * particleData_in,
                              ParticleData_F_t * particleDataF_out)
    {
      particleDataF_out->duty_cycle_pc = ((float) particleData_in->duty_cycle_pc_int)
                          + (((float) particleData_in->duty_cycle_pc_fr_2dp) / 100.0f);
      particleDataF_out->concentration = ((float) particleData_in->concentration_int)
                          + (((float) particleData_in->concentration_fr_2dp) / 100.0f);
      particleDataF_out->valid = (particleData_in->valid == 1);
    }


  };

} // namespace metriful_ms430
} // namespace esphome