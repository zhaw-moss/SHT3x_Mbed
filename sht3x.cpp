/*
 *           ______   _    ___        __          ___ ____   ____
 *          |__  / | | |  / \ \      / /         |_ _/ ___| / ___|
 *            / /| |_| | / _ \ \ /\ / /   _____   | |\___ \| |
 *           / /_|  _  |/ ___ \ V  V /   |_____|  | | ___) | |___
 *          /____|_| |_/_/   \_\_/\_/            |___|____/ \____|
 *
 *                Zurich University of Applied Sciences
 *         Institute of Signal Processing and Wireless Communications
 * ----------------------------------------------------------------------------
 */

/**
 * \file sht3x.cpp
 * \brief	Library for reading sensor data from the sensirion SHT3x series.
 * based on the coding example of sensirion
 *
 * \author	Simon Moser (moss)
 * \version	0.1
 */

#include "sht3x.h"
#include <cstdint>

Sht3x::Sht3x(I2C *_i2c, uint8_t _addr) {
  i2c = _i2c;
  addr = _addr;
  addr8 = addr << 1;
}

Sht3x::~Sht3x() {}

///-- public functions --------------------------------------------------------

void Sht3x::SetAddr(uint8_t _addr) {
  addr = _addr;
  addr8 = addr << 1;
}

sht3xError Sht3x::ReadSerialNumber(uint32_t *serialNumber) {
  sht3xError error; // error code
  uint16_t serialNumWords[2];

  error = WriteCommand(SHT3X_READ_SERIALNBR);
  // if no error, read first serial number word
  if (error == NO_ERROR)
    error = ReadBytesAndCrc(serialNumWords, 2);

  // if no error, calc serial number as 32-bit integer
  if (error == NO_ERROR) {
    *serialNumber = (serialNumWords[0] << 16) | serialNumWords[1];
  }

  return error;
}

sht3xError Sht3x::ReadStatus(uint16_t *status) {
  sht3xError error; // error code

  // write "read status" command
  error = WriteCommand(SHT3X_READ_STATUS);
  // if no error, read status
  if (error == NO_ERROR)
    error = ReadBytesAndCrc(status);

  return error;
}

sht3xError Sht3x::ClearAllAlertFlags(void) {
  sht3xError error; // error code

  // write clear status register command
  error = WriteCommand(SHT3X_CLEAR_STATUS);

  return error;
}

sht3xError Sht3x::GetTempAndHumi(float *temperature, float *humidity,
                                 sht3xRepeatability repeatability,
                                 sht3xMode mode, uint8_t timeout) {
  sht3xError error;

  switch (mode) {
  case MODE_CLKSTRETCH: // get temperature with clock stretching mode
    error = GetTempAndHumiClkStretch(temperature, humidity, repeatability);
    break;
  case MODE_POLLING: // get temperature with polling mode
  default:
    error = PARM_ERROR;
    break;
  }

  return error;
}

/// \todo Implement function StartPeriodicMeasurement

/// \todo Implement function ReadMeasurementBuffer

/// \todo Implement function EnableHeater

/// \todo Implement function DisableHeater

/// \todo Implement function SetAlertLimits

/// \todo Implement function GetAlertLimits

/// \todo Implement function ReadAlert

/// \todo Implement function SoftReset

/// \todo Implement function HardReset

///-- private functions -------------------------------------------------------

sht3xError Sht3x::GetTempAndHumiClkStretch(float *temperature, float *humidity,
                                           sht3xRepeatability repeatability) {
  sht3xError error;      // error code
  uint16_t rawValues[2]; // raw values from sensor

  // start measurement in clock stretching mode
  // use depending on the required repeatability, the corresponding command
  switch (repeatability) {
  case REPEATAB_LOW:
    error = WriteCommand(SHT3X_MEAS_CLOCKSTR_L);
    break;
  case REPEATAB_MEDIUM:
    error = WriteCommand(SHT3X_MEAS_CLOCKSTR_M);
    break;
  case REPEATAB_HIGH:
    error = WriteCommand(SHT3X_MEAS_CLOCKSTR_H);
    break;
  default:
    error = PARM_ERROR;
    break;
  }

  // if no error, read temperature raw values
  // if (error == NO_ERROR)
  error = ReadBytesAndCrc(rawValues, 2);

  // if no error, calculate temperature in degC and humidity in %RH
  if (error == NO_ERROR) {
    *temperature = CalcTemperature(rawValues[0]);
    *humidity = CalcHumidity(rawValues[1]);
  }

  // return error;
  return NO_ERROR;
}

sht3xError Sht3x::WriteCommand(sht3xCommands command) {
  const char cmd[2] = {static_cast<char>(command >> 8),
                       static_cast<char>(command & 0xFF)};
  int i2cErr = i2c->write(addr8, cmd, sizeof(cmd));
  return ConvertI2cError(i2cErr, WRITE);
}

sht3xError Sht3x::ReadBytesAndCrc(uint16_t *data, uint8_t n) {
  // check if the array size matches the given number
  if (n != sizeof(data) / 2) {
    return PARM_ERROR;
  }

  sht3xError error;
  uint8_t size = 3 * n; // total number of bytes to be read
  char bytes[size];     // buffer

  // read data
  int i2cErr = i2c->read(addr8, bytes, size);
  error = ConvertI2cError(i2cErr, READ);

  // verify checksum
  // if (error == NO_ERROR) {}
  // error = CheckCrc(bytes, 2, bytes[2]);}

  // store the data to the array
  for (uint8_t i = 0; i < n; i++) {
    data[i] = (bytes[3 * i] << 8) | bytes[3 * i + 1];
  }

  return error;
}

sht3xError Sht3x::CheckCrc(char *data, uint8_t len, char checksum) {
  uint32_t crc;                               // calculated checksum
  MbedCRC<0x31, 8> cs(0xFF, 0, false, false); // checksum instance

  // calculates 8-Bit checksum
  crc = cs.compute(data, len, &crc);

  // verify checksum
  if (crc != checksum) {
    return CHECKSUM_ERROR;
  } else {
    return NO_ERROR;
  }
}

float Sht3x::CalcTemperature(uint16_t rawValue) {
  // calculate temperature [degC]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

float Sht3x::CalcHumidity(uint16_t rawValue) {
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * (float)rawValue / 65535.0f;
}

sht3xError Sht3x::ConvertI2cError(uint8_t i2cErr, I2cDir dir) {
  sht3xError error;

  switch (dir) {
  case 0: // check write error
    switch (i2cErr) {
    case 0:
      error = NO_ERROR;
      break;

    case 1:
      error = NO_ERROR;
      break;

    case 2:
      error = TIMEOUT_ERROR;
      break;

    default:
      error = PARM_ERROR;
    }
    break;
  case 1: // check read error
    switch (i2cErr) {
    case 0:
      error = NO_ERROR;
      break;
    default:
      error = ACK_ERROR;
    }
  }

  return error;
}