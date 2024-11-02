# Metriful MS430: ESPHome External Component

The Metriful MS430 is a low-power, high-accuracy, smart sensor cluster for indoor environment monitoring. It operates via a simple I2C-compatible interface and measures eighteen variables, including air quality, light, and sound levels.

This repository provides instructions and software examples for integrating the MS430 with **ESPHome** as an external component. This approach simplifies integration, requiring only a few lines in the YAML file without needing a local ESPHome installation.

Please refer to the [Metriful repository](https://github.com/metriful/sensor) for detailed information about the sensor itself. This repository focuses solely on the ESPHome integration.

## ESPHome for Home Assistant

Send sensor data to an instance of [Home Assistant](https://www.home-assistant.io) using the [ESPHome system](https://esphome.io).

These instructions assume you have an existing Home Assistant installation and a basic understanding of ESPHome.

To add the Metriful MS430 to your ESPHome peripherals, proceed as follows:

1. Connect the MS430's I2C SDA and SCL pins to two suitable pins on the ESP.
2. Connect the MS430's RDY pin to a free GPIO pin on your ESP.
3. Add this repository as an external component in your ESPHome YAML file.
4. Configure the I2C settings in your ESPHome YAML file with the pins used for the MS430.
5. Specify the GPIO pin for the RDY line in the Metriful sensor configuration.
6. Let ESPHome update your device.

Example configuration:

Assuming I2C is connected to pins 19 and 22, and RDY is connected to pin 23, your YAML configuration would look like this:

```yaml
# Reference this repository so that ESPHome can use the code here
external_components:
  - source: 
      type: git
      url: https://github.com/SimoneGianni/metriful-esphome

# Configure I2C on the proper pins (may already be set up for other peripherals)
i2c:
  sda: 19
  scl: 22

# Metriful sensor setup, specifying the ready_pin, which is required
metriful:
  ready_pin: 23
  cycle_time: 100 # Optional, specity cycle time, can be 3, 100 or 300
```

All available sensors (temperature, humidity, CO2, air quality, sound levels in various bands, etc.) will be reported in Home Assistant and updated by default every 100 seconds. Note that certain measurements, like air quality and CO2, may require some time to initialize.

You can customize all sensor configurations to fit your needs:

```
TODO example of customization
```

## Troubleshooting

Direct support for individual problems is not provided. Please refer to the [Metriful repository](https://github.com/metriful/sensor) for troubleshooting the sensor and its compatibility with ESP platforms. If you encounter issues specific to the ESPHome integration, feel free to open an issue or submit a pull request.

## Changelog

Changes, fixes, and additions for each software release version are listed in the [CHANGELOG](CHANGELOG.md).

## License

See the [LICENSE](LICENSE.txt) file for software license rights and limitations (MIT).

## Disclaimer

This document, repository, and the products described herein are subject to specific disclaimers set forth in the [DISCLAIMER](DISCLAIMER.txt) file.
```

### Key improvements:
1. **Grammar and Consistency**: Adjusted phrasing for conciseness and flow.
2. **Clarity**: Made steps clear and distinct, improving readability.
3. **Terminology**: Adjusted terminology to ensure accuracy and precision, especially around "ESPHome peripheral" vs. "ESP device."

Let me know if you’d like further customization or detail!