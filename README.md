# A CO2 / Temperature / Humidity Sensor for ESP32 and ESP8622 Platforms

## Overview

The sensor can be used to measure the CO2 content, temperature and humidity inside closed rooms.
The measured data can be read directly from the sensor via the integrated web interface.
Alternatively, the measured values can be sent to any web server or cloud provider via MQTT or HTTP.

# Required Hardware Components

To build the sensor, the following components are needed:

- [ESP32 or ESP8622 board with integrated wifi antenna](https://www.espressif.com/en/products/devkits)
- [MH-Z19 CO2 sensor](https://esphome.io/components/sensor/mhz19.html)
- [DHT11 temperature and humidity sensor](https://learn.adafruit.com/dht)
- [Mini traffic light](https://www.az-delivery.de/en/products/led-ampel-modul)
- [Pack of jumber wires](https://www.az-delivery.de/en/products/mini-breadboard)
- [Mini breadboard](https://www.az-delivery.de/en/products/mini-breadboard)

# Firmware

The firmware is based on [Mongoose OS](https://mongoose-os.com/mos.html) and can be used freely.
See [Mongoose OS C/C++ quickstart](https://mongoose-os.com/docs/mongoose-os/quickstart/develop-in-c.md) for the C/C++ development introduction.
# User Interface

Once the firmware has been installed, the device acts as an access point to which you can connect via smartphone, for example.
Once you are connected to the network, you can access the device's web interface via a browser.
All you have to do is enter the IP address of the device in the browser bar.

# Miefalarm Cloud

