# Metriful MS430: ESP Home external component

The Metriful MS430 is a low power, high accuracy, smart sensor cluster for indoor environment monitoring. It operates via a simple I2C-compatible interface and measures eighteen variables including air quality, light and sound levels.

This repository provides instructions and software examples for using the MS430 in an **ESP Home** system as an external component, so that integrating it is just few lines in the YAML and does not require a local ESP Home installation. 

Please refer to [Metriful repository](https://github.com/metriful/sensor) for more in depth information about the sensor itself, this repository is only about the ESP Home integration. 


## ESPHome for Home Assistant

Send sensor data to an installation of [Home Assistant](https://www.home-assistant.io) using the [ESPHome system](https://esphome.io).

These instructions assume that you already have a working Home Assistant installation and you know what ESP Home is and how to stup a basic ESP Home peripheral.

To add the Metriful MS430 to your ESP Home perfipherals proceed as follows:

1. Connect the MS430 I2C SDA and SCL pins to two suitable pins of the ESP.

2. Connect the MS430 RDY pin to a free GPIO pin on your ESP.

4. Add to your ESP Home YAML file the external component refercing this repository.

5. Configure the ESP Home I2C settings in your ESP Home YAML file with the pins you used to connect the MS430.

6. Configure the Metriful sensor in you ESP Home YAML with the pin you used for the RDY line.

7. Let ESP Home update you device.

Done.

For example, having I2C on pins 19 and 22, and the RDY connected to pin 23, this is the resulting YAML snippet.

```yaml
# Reference this repository so that ESP Home can use the code here
external_components:
  - source: 
      type: git
      url: https://github.com/SimoneGianni/metriful-esphome

# Configure I2C on the proper pins (may be already setup for other peripherals)
i2c:
  sda: 19
  scl: 22

# Metriful sensor setup, specify the ready_pin, that's required
metriful:
  ready_pin: 23
```

All the available sensors (temperature, humidity, CO2, air quality, sound in various bands etc..) will be reported on your Home Assistant, and updated by default every 100 seconds; notice that air quality ones will need some time to initialize and report values, this includes CO2 and others. 

You can customize all the sensors as you want:

```
TODO example of customization
```

## Troubleshooting

I cannot support users problems, so please refer to [Metriful repository](https://github.com/metriful/sensor) for troubleshooting the sensor and it's working with an ESP platform. Report issues if you're sure it is a problem with the ESP Home integration, and obviously feel free to also provide a PR.


## Changelog

Changes, fixes and additions in each software release version are listed in the [CHANGELOG](CHANGELOG.md)


## License

See the [LICENSE](LICENSE.txt) file for software license rights and limitations (MIT).


## Disclaimer

This document and repository, and the products described herein, are subject to specific disclaimers set forth in the [DISCLAIMER](DISCLAIMER.txt) file.
