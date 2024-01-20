/*****************************************************************************
 *
 * Arduino driver for I2C temperature/humidity sensors HDC10X0
 *
 * file:     TI_HDC10XX.cpp
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

#include "TI_HDC10XX.h"

//#include <cfloat>

//#define DEBUG


TI_HDC10XX::TI_HDC10XX(uint8_t i2cAddress, TwoWire& wire) : wire(wire), i2cAddress(i2cAddress)
{
  // clear configuration register and preset default acquisition times
  init();
}

void TI_HDC10XX::init()
{
  // clear configuration register (assuming power-on reset)
  configurationRegister.reg = 0;

  // preset default acquisition times
  setResolution(14, 14);
}

void TI_HDC10XX::setBlocking(bool enabled)
{
  blocking = enabled;
}

bool TI_HDC10XX::isBlocking() const
{
  return blocking;
}

bool TI_HDC10XX::isConnected()
{
  // test if the devices replies
  size_t length = 1;
  uint8_t receivedLength = wire.requestFrom(i2cAddress, length);
#ifdef DEBUG
  if (receivedLength != length)
  {
    Serial.println("device is not connected");
  }
#endif
  return receivedLength == length;
}

bool TI_HDC10XX::reset()
{
  // clear configuration register and preset default acquisition times
  init();

  // reset device
  configurationRegister.bit.softwareReset = 1;
  bool success = writeRegister(REG_CONFIGURATION, configurationRegister.reg);
  configurationRegister.bit.softwareReset = 0;

  if (success && blocking)
  {
    // HDC1080 seems to require at least 8 ms to become fully ready after reset
    delay(8);
  }

  return success;
}

/**
 * temperature: 11 bit 3.65 ms, 14 bit 6.35 ms (default)
 * humidity:     8 bit 2.5 ms,  11 bit 3.85 ms, 14 bit 6.5 ms (default)
 * @return false if bit combination is not valid or configuration change failed
 */
bool TI_HDC10XX::setResolution(uint8_t temperatureBits, uint8_t humidityBits)
{
  bool valid = true;

#ifdef DEBUG
  if (humidityAcquisitionTime) Serial.println("setResolution");
#endif

  uint8_t temperatureMeasurementResolution = 0;
  uint16_t newTemperatureAcquisitionTime = 0; // [µs]
  switch (temperatureBits)
  {
    case 11:
      temperatureMeasurementResolution = 1;
      newTemperatureAcquisitionTime = 3650;
      break;
    case 14:
      temperatureMeasurementResolution = 0;
      newTemperatureAcquisitionTime = 6350;
      break;
    default:
      valid = false;
  }

  uint8_t humidityMeasurementResolution = 0;
  uint16_t newHumidityAcquisitionTime = 0; // [µs]
  switch (humidityBits)
  {
    case 8:
      humidityMeasurementResolution = 2;
      newHumidityAcquisitionTime = 2500;
      break;
    case 11:
      humidityMeasurementResolution = 1;
      newHumidityAcquisitionTime = 3850;
      break;
    case 14:
      humidityMeasurementResolution = 0;
      newHumidityAcquisitionTime = 6500;
      break;
    default:
      valid = false;
  }

  if (valid)
  {
#ifdef DEBUG
    if (humidityAcquisitionTime) Serial.println("setResolution: parameters are valid");
#endif
    temperatureAcquisitionTime = newTemperatureAcquisitionTime;
    humidityAcquisitionTime = newHumidityAcquisitionTime;
    ConfigurationRegister oldConfigurationRegister = configurationRegister;
    configurationRegister.bit.temperatureMeasurementResolution = temperatureMeasurementResolution;
    configurationRegister.bit.humidityMeasurementResolution = humidityMeasurementResolution;
    if (configurationRegister.reg != oldConfigurationRegister.reg)
    {
      return writeRegister(REG_CONFIGURATION, configurationRegister.reg);
    }
  }

  return valid;
}

bool TI_HDC10XX::setAcquisitionMode(AcquisitionMode mode)
{
  ConfigurationRegister oldConfigurationRegister = configurationRegister;
  configurationRegister.bit.modeOfAcquisition = mode;
  bool success = true;
  if (configurationRegister.reg != oldConfigurationRegister.reg)
  {
    success = writeRegister(REG_CONFIGURATION, configurationRegister.reg);
  }
  return success;
}

TI_HDC10XX::AcquisitionMode TI_HDC10XX::getAcquisitionMode() const
{
  return configurationRegister.bit.modeOfAcquisition ? ACQ_MODE_COMBINED : ACQ_MODE_SEPARATE;
}

uint16_t TI_HDC10XX::getAcquisitionTime() const
{
  switch (acquisitionType)
  {
    case ACQ_TYPE_TEMPERATURE:
      return temperatureAcquisitionTime;
    case ACQ_TYPE_HUMIDITY:
      return humidityAcquisitionTime;
    case ACQ_TYPE_COMBINED:
      return temperatureAcquisitionTime + humidityAcquisitionTime;
    default:
      return 0;
  }
}

bool TI_HDC10XX::setHeaterEnabled(bool enabled)
{
  ConfigurationRegister oldConfigurationRegister = configurationRegister;
  configurationRegister.bit.heater = enabled ? 1 : 0;
  bool success = true;
  if (configurationRegister.reg != oldConfigurationRegister.reg)
  {
    success = writeRegister(REG_CONFIGURATION, configurationRegister.reg);
  }
  return success;
}

bool TI_HDC10XX::isHeaterEnabled() const
{
  return configurationRegister.bit.heater;
}

bool TI_HDC10XX::heat(uint32_t milliseconds)
{
  if (milliseconds >= getAcquisitionTime())
  {
    // backup state
    bool wasHeaterEnabled = isHeaterEnabled();
    bool wasBlocking = blocking;

    // perform continuous acquisitions to heat the sensor
    setHeaterEnabled(true);
    blocking = true;
    uint32_t start = millis();
    bool success = true;
    while (delta(start, millis()) < milliseconds && success) success = startAcquisition(ACQ_TYPE_COMBINED);

    // restore state
    blocking = wasBlocking;
    setHeaterEnabled(wasHeaterEnabled);

    return success;
  }
  else
  {
    // duration is too short
    return false;
  }
}

bool TI_HDC10XX::isSupplyVoltageOK()
{
  ConfigurationRegister configurationRegister;
  bool statusOk = readRegister(REG_CONFIGURATION, configurationRegister.reg);
  if (statusOk)
  {
    this->configurationRegister = configurationRegister;
    statusOk = !configurationRegister.bit.batteryStatus;
  }
  return statusOk;
}

uint32_t TI_HDC10XX::readSerialIdLow()
{
  uint32_t serialId = 0;
  uint16_t raw[2];
  bool success = readRegister(REG_SERIAL_ID_1, raw[0]);
  if (success) success = readRegister(REG_SERIAL_ID_2, raw[1]);
  if (success)
  {
    serialId = raw[0] + ((uint32_t)raw[1] << 16);
  }
  return serialId;
}

uint32_t TI_HDC10XX::readSerialIdHigh()
{
  uint32_t serialId = 0;
  uint16_t raw[1];
  bool success = readRegister(REG_SERIAL_ID_3, raw[0]);
  if (success)
  {
    serialId = raw[0];
  }
  return serialId;
}

bool TI_HDC10XX::startAcquisition(uint8_t reg, AcquisitionType acquisitionType)
{
  bool success = true;
  if (!acquisitionStarted)
  {
    if (acquisitionType == ACQ_TYPE_COMBINED && !configurationRegister.bit.modeOfAcquisition)
    {
      // combined acquisition requested, but in separate acquisition mode, change
      success = setAcquisitionMode(ACQ_MODE_COMBINED);
    }

    if (success)
    {
      if (configurationRegister.bit.modeOfAcquisition == ACQ_MODE_SEPARATE)
      {
        // separate acquisition mode, start selected acquisition
        success = writeCommand(reg);
        if (success) this->acquisitionType = acquisitionType;
      }
      else
      {
        // combined acquisition mode, always start temperature acquisition
        success = writeCommand(REG_TEMPERATURE);
        if (success) this->acquisitionType = ACQ_TYPE_COMBINED;
      }
    }
    if (success)
    {
      // get acquisition start time
      acquisitionStarted = millis();
      dataState = DATA_STATE_INVALID;
    }
  }
  else
  {
    // acquisition in progress
    success = this->acquisitionType == acquisitionType || this->acquisitionType == ACQ_TYPE_COMBINED;
  }
  return success;
}

bool TI_HDC10XX::startAcquisition(AcquisitionType acquisitionType)
{
  if (blocking)
  {
    // start acquisition
    blocking = false;
    bool success = startAcquisition(acquisitionType);
    blocking = true;

    // wait for acquisition to complete
    if (success)
    {
      int available = 20; // [ms] timeout
      while (!isAcquisitionComplete() && (available-- > 0))
      {
        delay(1);
      }
      success = available;
    }

    // retrieve acquisition data from device
    if (success)
    {
      readAcquisitionData();
    }

    return success;
  }
  else
  {
    // non-blocking acquisition
    switch (acquisitionType)
    {
      case ACQ_TYPE_TEMPERATURE:
        return startAcquisition(REG_TEMPERATURE, acquisitionType);
      case ACQ_TYPE_HUMIDITY:
        return startAcquisition(REG_HUMIDITY, acquisitionType);
      case ACQ_TYPE_COMBINED:
        return startAcquisition(REG_TEMPERATURE, acquisitionType);
      default:
        return false;
    }
  }
}

void TI_HDC10XX::setDeepSleepDuration(uint16_t milliseconds)
{
  if (acquisitionStarted > 0)
  {
    acquisitionStarted += milliseconds;
  }
}

/**
 * @return true when acquisition delay has expired until humidity or temperature is read
 */
bool TI_HDC10XX::isAcquisitionComplete()
{
  bool available = false;
  uint16_t acquisitionTime = getAcquisitionTime(); // [µs]
  if (acquisitionTime > 0)
  {
    available = delta(acquisitionStarted, millis()) >= (1500U + acquisitionTime)/1000; // round up to next milli and add a little tolerance
  }
  return available;
}

bool TI_HDC10XX::isHumidityReady()
{
  if (acquisitionType == ACQ_TYPE_HUMIDITY || acquisitionType == ACQ_TYPE_COMBINED)
  {
    return isAcquisitionComplete();
  }
  else
  {
    return false;
  }
}

bool TI_HDC10XX::isTemperatureReady()
{
  if (acquisitionType == ACQ_TYPE_TEMPERATURE || acquisitionType == ACQ_TYPE_COMBINED)
  {
    return isAcquisitionComplete();
  }
  else
  {
    return false;
  }
}

bool TI_HDC10XX::readAcquisitionData()
{
  bool success = false;

  if (acquisitionType != ACQ_TYPE_NONE)
  {
    // read acquisition data
    uint8_t expectedSize = acquisitionType == ACQ_TYPE_COMBINED ? 4 : 2;
    uint8_t resultOffset = acquisitionType == ACQ_TYPE_HUMIDITY ? 2 : 0;
    success = readData(&acquisitionData[resultOffset], expectedSize);

    // qualify received data
    if (success)
    {
      switch (acquisitionType)
      {
        case DATA_STATE_HUMIDITY:
          dataState = DATA_STATE_HUMIDITY;
          break;
        case DATA_STATE_TEMPERATURE:
          dataState = DATA_STATE_TEMPERATURE;
          break;
        case ACQ_TYPE_COMBINED:
          dataState = DATA_STATE_COMBINED;
          break;
        default:
          dataState = DATA_STATE_INVALID;
      }
    }

    // clear acquisition state
    acquisitionStarted = 0;
    acquisitionType = ACQ_TYPE_NONE;
  }
  else if (dataState != DATA_STATE_INVALID)
  {
    success = true;
  }

  return success;
}

bool TI_HDC10XX::readHumidity()
{
  if (acquisitionType == ACQ_TYPE_HUMIDITY || acquisitionType == ACQ_TYPE_COMBINED || dataState == DATA_STATE_HUMIDITY || dataState == DATA_STATE_COMBINED)
  {
    return readAcquisitionData();
  }
  else
  {
    return false;
  }
}

bool TI_HDC10XX::readTemperature()
{
  if (acquisitionType == ACQ_TYPE_TEMPERATURE || acquisitionType == ACQ_TYPE_COMBINED || dataState == DATA_STATE_TEMPERATURE || dataState == DATA_STATE_COMBINED)
  {
    return readAcquisitionData();
  }
  else
  {
    return false;
  }
}

float TI_HDC10XX::getHumidity()
{
  if (dataState == DATA_STATE_HUMIDITY || dataState == DATA_STATE_COMBINED)
  {
    uint16_t rawHumidity = acquisitionData[2] << 8 | acquisitionData[3];
    return 100.0f*rawHumidity/65536U;
  }
  else
  {
    return 0;
  }
}

float TI_HDC10XX::getTemperature()
{
  if (dataState == DATA_STATE_TEMPERATURE || dataState == DATA_STATE_COMBINED)
  {
    uint16_t rawTemperature = acquisitionData[0] << 8 | acquisitionData[1];
    return 165.0f*rawTemperature/65536U - 40;
  }
  else
  {
    return -99;
  }
}

bool TI_HDC10XX::writeCommand(uint8_t reg)
{
#ifdef DEBUG
  Serial.print("writeCommand: ");
  Serial.println(reg);
#endif
  wire.beginTransmission(i2cAddress);
  wire.write(reg);
  uint8_t result = wire.endTransmission();
#ifdef DEBUG
  if (result)
  {
    Serial.print("writeCommand error: ");
    Serial.println(result);
  }
#endif
  return result == 0;
}

bool TI_HDC10XX::writeRegister(uint8_t reg, uint16_t data)
{
#ifdef DEBUG
  Serial.print("writeRegister: ");
  Serial.print(reg);
  Serial.print(" ");
  Serial.println(data);
#endif
  wire.beginTransmission(i2cAddress);
  wire.write(reg);
  wire.write(data >> 8);   // MSB
	wire.write(data & 0xFF); // LSB
  uint8_t result = wire.endTransmission();
#ifdef DEBUG
  if (result)
  {
    Serial.print("writeRegister error: ");
    Serial.println(result);
  }
#endif
  return result == 0;
}

bool TI_HDC10XX::readData(uint8_t data[], uint8_t length)
{
#ifdef DEBUG
  Serial.print("readData: ");
  Serial.print(length);
#endif
  int receivedLength = wire.requestFrom(i2cAddress, length);
  if (receivedLength == length)
  {
    wire.readBytes(data, length);
  }
#ifdef DEBUG
  Serial.print("/");
  Serial.print(receivedLength);
  for (int i=0; i<receivedLength; i++)
  {
    Serial.print("-");
    Serial.print(data[i]);
  }
  Serial.println();
#endif
  return receivedLength == length;
}

bool TI_HDC10XX::readRegister(uint8_t reg, uint16_t& data)
{
#ifdef DEBUG
  Serial.print("readRegister: ");
  Serial.println(reg);
#endif
  bool success = writeCommand(reg);
  if (success)
  {
    uint8_t buffer[2];
    success = readData(buffer, 2);
    if (success)
    {
      data = buffer[0] << 8 | buffer[1];
    }
    else
    {
      data = 0;
    }
  }
#ifdef DEBUG
  Serial.print("readRegister: ");
  Serial.print(reg);
  Serial.print("=");
  Serial.print(data);
  Serial.print("|");
  Serial.println(success);
#endif
  return success;
}

/**
 * calculates delta between start and end, assuming that end is always
 * after start and taking into account that there might have been  an
 * uint32_t overflow, so that the result is alway non-negative
 */
uint32_t TI_HDC10XX::delta(uint32_t start, uint32_t end)
{
  if (end >= start) return end - start;
  else              return UINT32_MAX - start + end;
}
