# Texas Instruments HDC10X0 Humidity and Temperature Sensor Driver for Arduino

## Features

- supports HDC1080
- supports HDC1000, HDC1008, HDC1010 and HDC1050 (untested)
- blocking and non-blocking mode
- multiple I2C bus support
- heater support
- convenience method to heat the sensor for a specified duration
- serial ID support
- CPU deep sleep support
- tested with SAMD21 MCU
- test build for ATmega 328P, ESP8266 and ESP32 MCU

### Table of contents

[1. Motivation](#motivation)  
[2. Results](#results)  
[3. Documentation](#documentation)  
[4. Examples](#examples)  
[5. Contributing](#contributing)  
[6. Licenses and Credits](#licenses-and-credits)

## Motivation

There are several Arduino libraries available for the HDC10XX sensors, but most of them share at least one of the following drawbacks:

- support for non-blocking usage missing
- build-in delays
- sparse implementation of device features
- not all HDC10XX variants supported
- GPL license

## Results

In non-blocking mode with 11 bit resolution it is possible to perform an acquisition cycle for humidity and temperature in about 10 ms with the CPU 
being involved for less than 1 ms (tested with a SAMD21 MCU). 

Freeing up the CPU for about 9 ms compared to blocking mode does not sound like much, but even with a CPU running at 8 MHz this typically equates to significantly more than 50.000 CPU instructions.

## Documentation

See method description in the [header file](src/TI_HDC10XX.h).

## Examples

Examples for blocking and non-blocking use can be found in the [examples](examples) subdirectory.

## Contributing

Contributors are welcome. Please create a merge request if you want to fix a bug or add a feature.

## Licenses and Credits

Copyright (c) 2024 [Jens B.](https://github.com/jnsbyr/)

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](http://www.apache.org/licenses/LICENSE-2.0)

This library depends on the Arduino platform and the Arduino Wire library.

The code was edited with [Visual Studio Code](https://code.visualstudio.com).

The badges in this document are provided by [img.shields.io](https://img.shields.io/).
