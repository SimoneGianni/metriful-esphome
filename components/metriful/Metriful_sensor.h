/*
  Metriful_sensor.h

  This file declares functions and settings which are used in the code
  examples. The function definitions are in file Metriful_sensor.cpp

  Copyright 2020-2023 Metriful Ltd.
  Licensed under the MIT License - for further details see LICENSE.txt

  For code examples, datasheet and user guide, visit
  https://github.com/metriful/sensor
*/

#ifndef METRIFUL_SENSOR_H
#define METRIFUL_SENSOR_H

#include "Arduino.h"
#include <Wire.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include "sensor_constants.h"

// Un-comment the following line to display temperatures in Fahrenheit
// else they will be in Celsius
//#define USE_FAHRENHEIT

// Specify which particle sensor is connected:
#define PARTICLE_SENSOR PARTICLE_SENSOR_OFF
// Define PARTICLE_SENSOR as:
//    PARTICLE_SENSOR_PPD42    for the Shinyei PPD42
//    PARTICLE_SENSOR_SDS011   for the Nova SDS011
//    PARTICLE_SENSOR_OFF      if no sensor is connected

// The I2C address of the MS430 board. 
#define I2C_ADDRESS I2C_ADDR_7BIT_SB_OPEN
// The default is I2C_ADDR_7BIT_SB_OPEN and must be changed to 
// I2C_ADDR_7BIT_SB_CLOSED if the solder bridge SB1 on the board 
// is soldered closed

/////////////////////////////////////////////////////////////////////

// The Arduino Wire library has a limited internal buffer size:
#define ARDUINO_WIRE_BUFFER_LIMIT_BYTES 32

#define I2C_CLK_FREQ_HZ 100000
#define SERIAL_BAUD_RATE 9600

// Unicode symbol strings
#define CELSIUS_SYMBOL "°C"
#define FAHRENHEIT_SYMBOL "°F"
#define SDS011_UNIT_SYMBOL "µg/m³"
#define SUBSCRIPT_2 "₂"
#define OHM_SYMBOL "Ω"

/////////////////////////////////////////////////////////////////////
// ISR_ATTRIBUTE was in host_pin_definition.h

#ifndef ARDUINO_PIN_DEFINITIONS_H
#define ARDUINO_PIN_DEFINITIONS_H

#ifdef ARDUINO_AVR_UNO

  // Arduino Uno

  #define ISR_ATTRIBUTE

#elif defined ARDUINO_SAMD_NANO_33_IOT

  // Arduino Nano 33 IoT

  #include <SPI.h>
  #include <WiFiNINA.h>
  #define HAS_WIFI
  #define ISR_ATTRIBUTE

#elif defined ARDUINO_AVR_NANO

  // Arduino Nano

  #define ISR_ATTRIBUTE

#elif defined ARDUINO_SAMD_MKRWIFI1010

  // Arduino MKR WiFi 1010

  #include <SPI.h>
  #include <WiFiNINA.h>
  #define HAS_WIFI
  #define ISR_ATTRIBUTE

#elif defined ESP8266

  // The examples have been tested on NodeMCU and Wemos D1 Mini.
  // Other ESP8266 boards may require changes.

  #include <ESP8266WiFi.h>
  #define HAS_WIFI
  #define ISR_ATTRIBUTE IRAM_ATTR

#elif defined ESP32

  // The examples have been tested on DOIT ESP32 DEVKIT V1 development board.
  // Other ESP32 boards may require changes.

  #include <WiFi.h>
  #define HAS_WIFI
  #define ISR_ATTRIBUTE IRAM_ATTR

#elif defined ARDUINO_ARCH_RP2040

  // The examples have been tested on the official Raspberry Pi Pico and
  // Pico W development boards. Other Pico/RP2040 boards may require changes.

  #ifdef ARDUINO_RASPBERRY_PI_PICO_W
    #include <WiFi.h>
    #define HAS_WIFI
  #endif

  #define ISR_ATTRIBUTE

#else
  #error ("Your development board is not yet supported.")
  // Please make a new section in this file to define
  // the correct input/output pins.
#endif

#endif


////

extern volatile bool ready_assertion_event; 

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

// Struct used in the IFTTT example
typedef struct
{
  const char * variableName;
  const char * measurementUnit;
  int32_t thresHigh;
  int32_t thresLow;  
  uint16_t inactiveCount;
  const char * adviceHigh;
  const char * adviceLow;
} ThresholdSetting_t;

// Struct used in the Home Assistant example
typedef struct
{
  const char * name;
  const char * unit;
  const char * icon;
  uint8_t decimalPlaces;
} HA_Attributes_t;

/////////////////////////////////////////////////////////////////////

#endif
