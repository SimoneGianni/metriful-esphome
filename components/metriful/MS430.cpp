/*
  Licensed under the MIT License - for further details see LICENSE.txt

  For code examples, datasheet and user guide, visit
  https://github.com/SimoneGianni/metriful-esphome
  https://github.com/metriful/sensor
*/

#include "MS430.h"

namespace esphome {
namespace metriful_ms430 {

static const char *const TAG = "ms430";

// ISR function to handle ready assertion event
volatile bool ready_assertion_event = true;
void ISR_ATTRIBUTE ready_ISR(void) {
  ready_assertion_event = true;
}

// Constructor
MS430::MS430() {
  // Initialize sound bands sensors
  for (uint8_t i = 0; i < SOUND_FREQ_BANDS; i++) {
    sound_bands_s[i] = new Sensor();
  }
}

// Dumps configuration details to the log
void MS430::dump_config() {
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

// Returns setup priority level
float MS430::get_setup_priority() const {
  return esphome::setup_priority::BUS;
}

// Initializes the MS430 device
void MS430::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MS430");

  pinMode(this->ready_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(this->ready_pin), ready_ISR, FALLING);

  /* Uncomment if particle sensor configuration is needed
  uint8_t particleSensor = PARTICLE_SENSOR;
  this->transmitI2C(PARTICLE_SENSOR_SELECT_REG, &particleSensor, 1);
  */
}

// Main loop function to handle communication and sensor data reading
void MS430::loop() {
  if ((last_report + ((cycle_time * 1000) + 10000)) < millis()) {
    ESP_LOGI(TAG, "Resetting for timeout");
    ready_assertion_event = true;
    comm_state = 0;
  }
  
  if (ready_assertion_event) {
    ESP_LOGD(TAG, "Got assertion event");
    ready_assertion_event = false;
    last_report = millis();

    if (cycle_time_changed) {
      comm_state = 0;
      cycle_time_changed = false;
    }

    uint8_t cyclePeriod;

    switch (comm_state) {
      case 0:
        ESP_LOGD(TAG, "Resetting");
        this->transmitI2C(RESET_CMD, nullptr, 0);
        comm_state = 1;
        break;
      case 1:
        ESP_LOGD(TAG, "Setting cycle time and mode");
        cyclePeriod = this->cycle_time >= 300 ? CYCLE_PERIOD_300_S :
                              this->cycle_time >= 100 ? CYCLE_PERIOD_100_S :
                              CYCLE_PERIOD_3_S;
        this->transmitI2C(CYCLE_TIME_PERIOD_REG, &cyclePeriod, 1);
        this->transmitI2C(CYCLE_MODE_CMD, nullptr, 0);
        comm_state = 100;
        break;
      default:
        readSensors();
        if (comm_state == 100) {
            comm_state++;
        }
    }
  }

  // Publish sensor data based on the current state
  if (comm_state > 100) {
    if (comm_state == 101) {
        temperature_s->publish_state(airDataF.T_C);
    } else if (comm_state == 102) {
        pressure_s->publish_state(airDataF.P_Pa);
    } else if (comm_state == 103) {
        humidity_s->publish_state(airDataF.H_pc);
    } else if (comm_state == 104) {
        gas_s->publish_state(airDataF.G_Ohm);
    } else if (comm_state == 105) {
        if (firstOutput) {
        aqi_acc_s->publish_state(-1.0);
        firstOutput = false;
        }
        aqi_acc_s->publish_state(airQualityDataF.AQI_accuracy);
    } else if (comm_state == 106) {
        if (airQualityDataF.AQI_accuracy > 0) {
        AQIinitialized = true;
        }
        if (AQIinitialized) {
        if (firstAQIoutput) {
            aqi_s->publish_state(-1.0);
            firstAQIoutput = false;
        }
        }
    } else if (comm_state == 107) {
        if (AQIinitialized) {
        aqi_s->publish_state(airQualityDataF.AQI);
        }
    } else if (comm_state == 108) {
        if (AQIinitialized) {
        CO2e_s->publish_state(airQualityDataF.CO2e);
        }
    } else if (comm_state == 109) {
        if (AQIinitialized) {
        bVOC_s->publish_state(airQualityDataF.bVOC);
        }
    } else if (comm_state == 110) {
        if (PARTICLE_SENSOR != PARTICLE_SENSOR_OFF) {
        particle_duty_s->publish_state(particleDataF.duty_cycle_pc);
        }
    } else if (comm_state == 111) {
        if (PARTICLE_SENSOR != PARTICLE_SENSOR_OFF) {
        particle_conc_s->publish_state(particleDataF.concentration);
        }
    } else if (comm_state == 112) {
        illuminance_s->publish_state(lightDataF.illum_lux);
    } else if (comm_state == 113) {
        w_light_s->publish_state(lightDataF.white);
    } else if (comm_state == 114) {
        sound_spl_s->publish_state(soundDataF.SPL_dBA);
    } else if (comm_state == 115) {
        sound_peak_s->publish_state(soundDataF.peakAmp_mPa);
    } else if (comm_state == 116) {
        sound_bands_s[0]->publish_state(soundDataF.SPL_bands_dB[0]);
    } else if (comm_state == 117) {
        sound_bands_s[1]->publish_state(soundDataF.SPL_bands_dB[1]);
    } else if (comm_state == 118) {
        sound_bands_s[2]->publish_state(soundDataF.SPL_bands_dB[2]);
    } else if (comm_state == 119) {
        sound_bands_s[3]->publish_state(soundDataF.SPL_bands_dB[3]);
    } else if (comm_state == 120) {
        sound_bands_s[4]->publish_state(soundDataF.SPL_bands_dB[4]);
    } else if (comm_state == 121) {
        sound_bands_s[5]->publish_state(soundDataF.SPL_bands_dB[5]);
    } else {
        comm_state = 99;
    }
    comm_state++;
  }
}

// Reads sensor data
void MS430::readSensors() {
  airDataF = getAirDataF();
  airQualityDataF = getAirQualityDataF();
  lightDataF = getLightDataF();
  soundDataF = getSoundDataF();
  if (PARTICLE_SENSOR != PARTICLE_SENSOR_OFF) {
    particleDataF = getParticleDataF();
  }
}

// I2C transmit helper function
bool MS430::transmitI2C(uint8_t commandRegister, const uint8_t *data, uint8_t data_length) {
  i2c::WriteBuffer buffers[2];
  buffers[0].data = &commandRegister;
  buffers[0].len = 1;

  if (data_length > 0) {
    buffers[1].data = data;
    buffers[1].len = data_length;
    ESP_LOGV(TAG, "Sending two buffers");
    return (bus_->writev(address_, buffers, 2, true) == i2c::ERROR_OK);
  } else {
    ESP_LOGV(TAG, "Sending command only");
    return (bus_->writev(address_, buffers, 1, true) == i2c::ERROR_OK);
  }
}

// I2C receive helper function
bool MS430::receiveI2C(uint8_t commandRegister, uint8_t data[], uint8_t data_length) {
  i2c::WriteBuffer buffer = {&commandRegister, 1};
  if (bus_->writev(address_, &buffer, 1, false) != i2c::ERROR_OK) {
    return false;
  }

  this->read(data, data_length);
  return true;
}

// Sensor data fetching functions
SoundData_t MS430::getSoundData() {
  SoundData_t soundData = {0};
  this->receiveI2C(SOUND_DATA_READ, (uint8_t *)&soundData, SOUND_DATA_BYTES);
  return soundData;
}

AirData_t MS430::getAirData() {
  AirData_t airData = {0};
  this->receiveI2C(AIR_DATA_READ, (uint8_t *)&airData, AIR_DATA_BYTES);
  return airData;
}

LightData_t MS430::getLightData() {
  LightData_t lightData = {0};
  this->receiveI2C(LIGHT_DATA_READ, (uint8_t *)&lightData, LIGHT_DATA_BYTES);
  return lightData;
}

AirQualityData_t MS430::getAirQualityData() {
  AirQualityData_t airQualityData = {0};
  this->receiveI2C(AIR_QUALITY_DATA_READ, (uint8_t *)&airQualityData, AIR_QUALITY_DATA_BYTES);
  return airQualityData;
}

ParticleData_t MS430::getParticleData() {
  ParticleData_t particleData = {0};
  this->receiveI2C(PARTICLE_DATA_READ, (uint8_t *)&particleData, PARTICLE_DATA_BYTES);
  return particleData;
}

////////////////////////////////////////////////////////////////////////

// Functions to convert data from integer representation to floating-point
// representation. Floats are easy to use for writing programs but require
// greater memory and processing power resources, so may not always be
// appropriate.

// Decode and convert the temperature as read from the MS430 (integer
// representation) into a float value 

float MS430::convertEncodedTemperatureToFloat(uint8_t T_C_int_with_sign, uint8_t T_C_fr_1dp) {
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


void MS430::convertAirDataF(const AirData_t * airData_in, AirData_F_t * airDataF_out) {
  // Decode the signed value for T (in Celsius)
  airDataF_out->T_C = convertEncodedTemperatureToFloat(
                        airData_in->T_C_int_with_sign,
                        airData_in->T_C_fr_1dp);
  airDataF_out->P_Pa = airData_in->P_Pa;
  airDataF_out->H_pc = ((float) airData_in->H_pc_int)
                        + (((float) airData_in->H_pc_fr_1dp) / 10.0f);
  airDataF_out->G_Ohm = airData_in->G_ohm;
}

void MS430::convertAirQualityDataF(const AirQualityData_t * airQualityData_in, AirQualityData_F_t * airQualityDataF_out) {
  airQualityDataF_out->AQI =  ((float) airQualityData_in->AQI_int) + 
                            (((float) airQualityData_in->AQI_fr_1dp) / 10.0f);
  airQualityDataF_out->CO2e = ((float) airQualityData_in->CO2e_int) + 
                            (((float) airQualityData_in->CO2e_fr_1dp) / 10.0f);
  airQualityDataF_out->bVOC = ((float) airQualityData_in->bVOC_int) + 
                            (((float) airQualityData_in->bVOC_fr_2dp) / 100.0f);
  airQualityDataF_out->AQI_accuracy = airQualityData_in->AQI_accuracy;
}

void MS430::convertLightDataF(const LightData_t * lightData_in, LightData_F_t * lightDataF_out) {
  lightDataF_out->illum_lux = ((float) lightData_in->illum_lux_int)
                          + (((float) lightData_in->illum_lux_fr_2dp) / 100.0f);
  lightDataF_out->white = lightData_in->white;
}

void MS430::convertSoundDataF(const SoundData_t * soundData_in, SoundData_F_t * soundDataF_out) {
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

void MS430::convertParticleDataF(const ParticleData_t * particleData_in, ParticleData_F_t * particleDataF_out) {
  particleDataF_out->duty_cycle_pc = ((float) particleData_in->duty_cycle_pc_int)
                      + (((float) particleData_in->duty_cycle_pc_fr_2dp) / 100.0f);
  particleDataF_out->concentration = ((float) particleData_in->concentration_int)
                      + (((float) particleData_in->concentration_fr_2dp) / 100.0f);
  particleDataF_out->valid = (particleData_in->valid == 1);
}

} // namespace metriful_ms430
} // namespace esphome
