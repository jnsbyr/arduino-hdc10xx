/*****************************************************************************
 *
 * Arduino driver for I2C temperature/humidity sensors HDC10X0
 *
 * file:     TI_HDC10XX.h
 * encoding: UTF-8
 * created:  12.01.2024
 *
 *****************************************************************************
 *
 * Copyright (C) 2024 Jens B.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *****************************************************************************/

#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * Arduino driver for Texas Instruments HDC10XX temperature and humidity sensors with I2C interface
 *
 * driver features:
 * - support HDC1080
 * - support HDC1000, HDC1008, HDC1010 and HDC1050 (untested)
 * - blocking and non-blocking mode
 * - multiple I2C bus support
 * - heater support
 * - convenience method to heat the sensor for a specified duration
 * - serial ID support
 * - CPU deep sleep support
 *
 * device features (HDC1080):
 * - capacitive hygrometer (dielectric polymer), see TI document snaa318 for more details
 * - factory calibrated
 * - typical humidity accuracy +/- 2 %
 * - typical temperature accuracy +/- 0.2 °C
 * - humidity acquisition time 2.5 .. 6.5 ms
 * - temperature acquisition time 3.65 .. 6.25 ms
 * - humidity measurement ambient temperature range -20 .. 85 °C
 * - temperature measurement ambient temperature range -40 .. 125 °C
 * - min. supply voltage: 2.7
 * - sleep current 0.1 µA
 * - humidity acquisition current 190 µA
 * - temperature acquisition current 160 µA
 * - heater current 7.2 mA
 *
 * dependencies:
 * - Adruino Wire library
 */
class TI_HDC10XX
{
public:
  enum AcquisitionMode
  {
    ACQ_MODE_SEPARATE = 0,
    ACQ_MODE_COMBINED = 1  // 1. temperature, 2. humidity
  };

  enum AcquisitionType
  {
    ACQ_TYPE_NONE        = 0, // for internal use
    ACQ_TYPE_TEMPERATURE = 1,
    ACQ_TYPE_HUMIDITY    = 2,
    ACQ_TYPE_COMBINED    = 3
  };

public:
  TI_HDC10XX(uint8_t i2cAddress, TwoWire& wire);

public:
  // make reset() and startAcquisition() block until operation is complete (default: disabled)
  void setBlocking(bool enabled);
  bool isBlocking() const;
  // check if device is connected, consider calling Wire.setTimeout() first
  bool isConnected();
  // reset device, device will need at least 8 ms to be completely ready in non-blocking mode
  bool reset();
  // set acquisition resolution
  // temperature: 11 bit 3.65 ms, 14 bit 6.35 ms (default)
  // humidity: 8 bit 2.5 ms, 11 bit 3.85 ms, 14 bit 6.5 ms (default)
  bool setResolution(uint8_t temperatureBits, uint8_t humidityBits);
  // acquisition mode: ACQ_MODE_SEPARATE (default), ACQ_MODE_COMBINED
  bool setAcquisitionMode(AcquisitionMode mode);
  AcquisitionMode getAcquisitionMode() const;
  // microseconds required for acquisition based on resolution and mode
  uint16_t getAcquisitionTime() const;
  // enable (7.2 mA during measurement) or disable (default) heater, requires periodic call to startAcquisition()
  bool setHeaterEnabled(bool enabled);
  bool isHeaterEnabled() const;
  // convenience method to enable heater for a specified duration, always blocking
  bool heat(uint32_t milliseconds);
  // check if device supply voltage is above 2.8 V, also updates configuration register state from device
  bool isSupplyVoltageOK();
  // get unique 40 bit serial ID from device, low word
  uint32_t readSerialIdLow();
  // get unique 40 bit serial ID from device, high word
  uint32_t readSerialIdHigh();

public:
  // request new acquisition: ACQ_TYPE_TEMPERATURE, ACQ_TYPE_HUMIDITY, ACQ_TYPE_COMBINED (will activate ACQ_MODE_COMBINED)
  // notes:
  // - selecting ACQ_TYPE_COMBINED will activate ACQ_MODE_COMBINED
  // - humidity acquisition in ACQ_MODE_SEPARATE requires at least one previous temperature acquisition, will otherwise return 99.9 %RH
  // - use isAcquisitionComplete() and readAcquisitionData() in non-blocking mode before calling getTemperature() or getHumidity()
  bool startAcquisition(AcquisitionType acquisitionType);

  // compensate CPU deep sleep duration after start acquisition (time where SysTick was not updated)
  void setDeepSleepDuration(uint16_t milliseconds);

  // check if acquisition is complete based on expired time since start
  bool isAcquisitionComplete();
  // check if humidity acquisition is complete
  bool isHumidityReady();
  // check if temperature acquisition is complete
  bool isTemperatureReady();

  // retrieve acquisition data from device, returns false if not ready
  bool readAcquisitionData();
  // retrieve temperature from device, returns false if not ready, will also retrieve humidity in ACQ_MODE_COMBINED
  bool readTemperature();
  // retrieve humidity from device, returns false if not ready, will also retrieve temperature in ACQ_MODE_COMBINED
  bool readHumidity();

  // return last retrieved temperature value [°C]
  float getTemperature();
  // return last retrieved humidity value [%RH]
  float getHumidity();

protected:
  enum Register
  {
    REG_TEMPERATURE     = 0x00,
    REG_HUMIDITY        = 0x01,
    REG_CONFIGURATION   = 0x02,
    REG_SERIAL_ID_1     = 0xFB,
    REG_SERIAL_ID_2     = 0xFC,
    REG_SERIAL_ID_3     = 0xFD,
    REG_MANUFACTURER_ID = 0xFE, // 0x5449
    REG_DEVICE_ID       = 0xFF  // 0x1050
  };

  enum DataState
  {
    DATA_STATE_INVALID     = 0,
    DATA_STATE_TEMPERATURE = 1,
    DATA_STATE_HUMIDITY    = 2,
    DATA_STATE_COMBINED    = 3
  };

  typedef union
  {
    struct {
      uint8_t :8;
      uint8_t humidityMeasurementResolution:2;
      uint8_t temperatureMeasurementResolution:1;
      uint8_t batteryStatus:1; // 0: >2.8V
      uint8_t modeOfAcquisition:1;
      uint8_t heater:1;
      uint8_t :1;
      uint8_t softwareReset:1;
    }  bit;
    uint16_t reg;
  } ConfigurationRegister;

protected:
  void init();
  bool startAcquisition(uint8_t reg, AcquisitionType acquisitionType);
  bool writeCommand(uint8_t reg);
  bool writeRegister(uint8_t reg, uint16_t data);
  bool readRegister(uint8_t reg, uint16_t& data);
  bool readData(uint8_t data[], uint8_t length);
  uint32_t delta(uint32_t start, uint32_t end);

protected:
  TwoWire& wire;
  ConfigurationRegister configurationRegister;
  AcquisitionType acquisitionType = ACQ_TYPE_NONE;
  DataState dataState = DATA_STATE_INVALID;
  uint32_t acquisitionStarted = 0; // [ms]
  uint16_t humidityAcquisitionTime = 0; // [µs]
  uint16_t temperatureAcquisitionTime = 0; // [µs]
  uint8_t i2cAddress;
  uint8_t acquisitionData[4];
  bool blocking = false;
};

class TI_HDC1000 : public TI_HDC10XX
{
public:
  // @param i2cAddress 0x40 .. 0x43, depending on address pin setting
  TI_HDC1000(uint8_t i2cAddress, TwoWire& wire = Wire) : TI_HDC10XX(i2cAddress, wire) {};
};

class TI_HDC1008 : public TI_HDC10XX
{
public:
  // @param i2cAddress 0x40 .. 0x43, depending on address pin setting
  TI_HDC1008(uint8_t i2cAddress, TwoWire& wire = Wire) : TI_HDC10XX(i2cAddress, wire) {};
};

class TI_HDC1010 : public TI_HDC10XX
{
public:
  // @param i2cAddress 0x40 .. 0x43, depending on address pin setting
  TI_HDC1010(uint8_t i2cAddress, TwoWire& wire = Wire) : TI_HDC10XX(i2cAddress, wire) {};
};

class TI_HDC1050 : public TI_HDC10XX
{
public:
  TI_HDC1050(TwoWire& wire = Wire) : TI_HDC10XX(0x40, wire) {};
};

class TI_HDC1080 : public TI_HDC10XX
{
public:
  TI_HDC1080(TwoWire& wire = Wire) : TI_HDC10XX(0x40, wire) {};
};
