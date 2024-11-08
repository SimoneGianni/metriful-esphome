/*
  sensor_constants.h

  Licensed under the MIT License - for further details see LICENSE.txt

  For code examples, datasheet and user guide, visit
  https://github.com/SimoneGianni/metriful-esphome
  https://github.com/metriful/sensor
*/

#ifndef SENSOR_CONSTANTS_H
#define SENSOR_CONSTANTS_H

#include <stdint.h>
#include <stdbool.h>

// Specify which particle sensor is connected:
#define PARTICLE_SENSOR PARTICLE_SENSOR_OFF
// Define PARTICLE_SENSOR as:
//    PARTICLE_SENSOR_PPD42    for the Shinyei PPD42
//    PARTICLE_SENSOR_SDS011   for the Nova SDS011
//    PARTICLE_SENSOR_OFF      if no sensor is connected

/////////////////////////////////////////////////////////////////////
// ISR_ATTRIBUTE was in host_pin_definition.h

#ifndef ARDUINO_PIN_DEFINITIONS_H
#define ARDUINO_PIN_DEFINITIONS_H

#ifdef ARDUINO_AVR_UNO

  // Arduino Uno

  #define ISR_ATTRIBUTE

#elif defined ARDUINO_SAMD_NANO_33_IOT

  // Arduino Nano 33 IoT

  #define ISR_ATTRIBUTE

#elif defined ARDUINO_AVR_NANO

  // Arduino Nano

  #define ISR_ATTRIBUTE

#elif defined ARDUINO_SAMD_MKRWIFI1010

  // Arduino MKR WiFi 1010

  #define ISR_ATTRIBUTE

#elif defined ESP8266

  // The examples have been tested on NodeMCU and Wemos D1 Mini.
  // Other ESP8266 boards may require changes.

  #define ISR_ATTRIBUTE IRAM_ATTR

#elif defined ESP32

  // The examples have been tested on DOIT ESP32 DEVKIT V1 development board.
  // Other ESP32 boards may require changes.

  #define ISR_ATTRIBUTE IRAM_ATTR

#elif defined ARDUINO_ARCH_RP2040

  // The examples have been tested on the official Raspberry Pi Pico and
  // Pico W development boards. Other Pico/RP2040 boards may require changes.

  #define ISR_ATTRIBUTE

#else
  #error ("Your development board is not yet supported.")
  // Please make a new section in this file to define
  // the correct input/output pins.
#endif

#endif



///////////////////////////////////////////////////////////
// Register and command addresses:

// Settings registers
#define PARTICLE_SENSOR_SELECT_REG 0x07

#define LIGHT_INTERRUPT_ENABLE_REG 0x81     
#define LIGHT_INTERRUPT_THRESHOLD_REG 0x82  
#define LIGHT_INTERRUPT_TYPE_REG 0x83       
#define LIGHT_INTERRUPT_POLARITY_REG 0x84   

#define SOUND_INTERRUPT_ENABLE_REG 0x85     
#define SOUND_INTERRUPT_THRESHOLD_REG 0x86  
#define SOUND_INTERRUPT_TYPE_REG 0x87     

#define CYCLE_TIME_PERIOD_REG 0x89

// Executable commands
#define ON_DEMAND_MEASURE_CMD 0xE1
#define RESET_CMD 0xE2
#define CYCLE_MODE_CMD 0xE4
#define STANDBY_MODE_CMD 0xE5
#define LIGHT_INTERRUPT_CLR_CMD 0xE6
#define SOUND_INTERRUPT_CLR_CMD 0xE7

// Read the operational mode
#define OP_MODE_READ 0x8A 

// Read data for whole categories 
#define AIR_DATA_READ   0x10
#define AIR_QUALITY_DATA_READ 0x11
#define LIGHT_DATA_READ 0x12
#define SOUND_DATA_READ 0x13
#define PARTICLE_DATA_READ 0x14

// Read individual data quantities
#define T_READ 0x21
#define P_READ 0x22
#define H_READ 0x23
#define G_READ 0x24

#define AQI_READ 0x25
#define CO2E_READ 0x26
#define BVOC_READ 0x27
#define AQI_ACCURACY_READ 0x28

#define ILLUMINANCE_READ 0x31
#define WHITE_LIGHT_READ 0x32

#define SPL_READ 0x41
#define SPL_BANDS_READ 0x42
#define SOUND_PEAK_READ 0x43
#define SOUND_STABLE_READ 0x44

#define DUTY_CYCLE_READ 0x51
#define CONCENTRATION_READ 0x52
#define PARTICLE_VALID_READ 0x53

///////////////////////////////////////////////////////////

// Sizes of data expected when setting interrupt thresholds
#define LIGHT_INTERRUPT_THRESHOLD_BYTES 3
#define SOUND_INTERRUPT_THRESHOLD_BYTES 2

// Frequency bands for sound level measurement
#define SOUND_FREQ_BANDS 6
static const uint16_t sound_band_mids_Hz[SOUND_FREQ_BANDS] =
                                {125, 250, 500, 1000, 2000, 4000};
static const uint16_t sound_band_edges_Hz[SOUND_FREQ_BANDS+1] =
                                {88, 177, 354, 707, 1414, 2828, 5657};

// Cycle mode time period
#define CYCLE_PERIOD_3_S    0
#define CYCLE_PERIOD_100_S  1
#define CYCLE_PERIOD_300_S  2

// Sound interrupt type:
#define SOUND_INT_TYPE_LATCH  0
#define SOUND_INT_TYPE_COMP   1

// Maximum for illuminance measurement and threshold setting
#define MAX_LUX_VALUE 3774

// Light interrupt type:
#define LIGHT_INT_TYPE_LATCH  0
#define LIGHT_INT_TYPE_COMP   1

// Light interrupt polarity:
#define LIGHT_INT_POL_POSITIVE  0
#define LIGHT_INT_POL_NEGATIVE  1

// Decoding the temperature integer.fraction value format
#define TEMPERATURE_VALUE_MASK 0x7F
#define TEMPERATURE_SIGN_MASK  0x80

// Particle sensor module selection:
#define PARTICLE_SENSOR_OFF    0
#define PARTICLE_SENSOR_PPD42  1
#define PARTICLE_SENSOR_SDS011 2

///////////////////////////////////////////////////////////

// Structs for accessing individual data quantities after
// reading a category of data.

typedef struct __attribute__((packed))
{
  uint8_t  T_C_int_with_sign;    
  uint8_t  T_C_fr_1dp; 
  uint32_t P_Pa;
  uint8_t  H_pc_int;
  uint8_t  H_pc_fr_1dp;
  uint32_t G_ohm;
} AirData_t;

typedef struct __attribute__((packed))
{
  uint16_t AQI_int;
  uint8_t  AQI_fr_1dp;
  uint16_t CO2e_int;
  uint8_t  CO2e_fr_1dp;
  uint16_t bVOC_int;
  uint8_t  bVOC_fr_2dp;
  uint8_t  AQI_accuracy;
} AirQualityData_t;

typedef struct __attribute__((packed))
{
  uint16_t illum_lux_int;
  uint8_t  illum_lux_fr_2dp;
  uint16_t white;
} LightData_t;

typedef struct __attribute__((packed))
{
  uint8_t  SPL_dBA_int;
  uint8_t  SPL_dBA_fr_1dp;
  uint8_t  SPL_bands_dB_int[SOUND_FREQ_BANDS];
  uint8_t  SPL_bands_dB_fr_1dp[SOUND_FREQ_BANDS];
  uint16_t peak_amp_mPa_int;
  uint8_t  peak_amp_mPa_fr_2dp;
  uint8_t  stable;
} SoundData_t;

typedef struct __attribute__((packed))
{
  uint8_t  duty_cycle_pc_int;
  uint8_t  duty_cycle_pc_fr_2dp;
  uint16_t concentration_int;
  uint8_t  concentration_fr_2dp;
  uint8_t  valid;
} ParticleData_t;


/////////////////////////////////////////////////////////////////////

// Data category structs containing floats. If floats are not wanted, 
// use the integer-only struct versions in sensor_constants.h 

typedef struct
{
  float SPL_dBA;
  float SPL_bands_dB[SOUND_FREQ_BANDS];
  float peakAmp_mPa;
  bool  stable;
} SoundData_F_t;

typedef struct
{
  float    T_C;
  uint32_t P_Pa;
  float    H_pc;
  uint32_t G_Ohm;
} AirData_F_t;

typedef struct
{
  float   AQI;
  float   CO2e;
  float   bVOC;
  uint8_t AQI_accuracy;
} AirQualityData_F_t;

typedef struct
{
  float    illum_lux;
  uint16_t white;
} LightData_F_t;

typedef struct
{
  float duty_cycle_pc;
  float concentration;
  bool valid;
} ParticleData_F_t;

/////////////////////////////////////////////////////////////////////

// Custom type used to select the particle sensor being used (if any)
typedef enum
{
  OFF    = PARTICLE_SENSOR_OFF,
  PPD42  = PARTICLE_SENSOR_PPD42,
  SDS011 = PARTICLE_SENSOR_SDS011
} ParticleSensor_t;


/////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////

// Byte lengths for each readable data quantity and data category

#define T_BYTES         2
#define P_BYTES         4
#define H_BYTES         2
#define G_BYTES         4
#define AIR_DATA_BYTES  sizeof(AirData_t)

#define AQI_BYTES               3
#define CO2E_BYTES              3
#define BVOC_BYTES              3
#define AQI_ACCURACY_BYTES      1
#define AIR_QUALITY_DATA_BYTES  sizeof(AirQualityData_t)

#define ILLUMINANCE_BYTES 3
#define WHITE_BYTES       2
#define LIGHT_DATA_BYTES  sizeof(LightData_t)

#define SPL_BYTES           2
#define SPL_BANDS_BYTES    (2*SOUND_FREQ_BANDS)
#define SOUND_PEAK_BYTES    3
#define SOUND_STABLE_BYTES  1
#define SOUND_DATA_BYTES    sizeof(SoundData_t)

#define DUTY_CYCLE_BYTES     2
#define CONCENTRATION_BYTES  3
#define PARTICLE_VALID_BYTES 1
#define PARTICLE_DATA_BYTES  sizeof(ParticleData_t)

#endif
