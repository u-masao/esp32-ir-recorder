ESP32-IR-Recorder
=================

ESP32-IR-Recorder can control TVs, video recorders, air conditioners, etc. with ESP32.

## Description

## Hardware
- ESP32
- Infrared sensor module --- connect GPIO12, GPIO13
- Infrared LED module --- connect GPIO14
  - Infrared LED 940nm 40mW/50mA
  - register 33[ohm]
  - FET (ex. IRFU9024NPBF)

## Requirement
- [ESP-IDF](https://github.com/espressif/esp-idf)

## Usage
- Turn on the ESP32.
- Point the remote controller for the TV you want to learn to the infrared sensor connected to ESP32.
- Press the button you want to record to ESP32.
- ESP32 sends the registered command every second.

