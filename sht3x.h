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
 * \file sht3x.h
 * \brief	Library for reading sensor data from the sensirion SHT3x series.
 * Based on the coding example of sensirion
 *
 * \author	Simon Moser (moss)
 * \version	0.1
 */

#ifndef _SHT3X_H
#define _SHT3X_H

#include <cstdint>
#include <drivers/I2C.h>
#include <drivers/MbedCRC.h>
#include <mbed.h>

/// Default i2c address
#define SHT3X_ADDR 0x44

//-- public enums -------------------------------------------------------------
/// Sensor Commands
typedef enum sht3xCommands {
  SHT3X_READ_SERIALNBR = 0x3780,  ///< read serial number
  SHT3X_READ_STATUS = 0xF32D,     ///< read status register
  SHT3X_CLEAR_STATUS = 0x3041,    ///< clear status register
  SHT3X_HEATER_ENABLE = 0x306D,   ///< enabled heater
  SHT3X_HEATER_DISABLE = 0x3066,  ///< disable heater
  SHT3X_SOFT_RESET = 0x30A2,      ///< soft reset
  SHT3X_MEAS_CLOCKSTR_H = 0x2C06, ///< clock stretching, high repeatability
  SHT3X_MEAS_CLOCKSTR_M = 0x2C0D, ///< clock stretching, medium repeatability
  SHT3X_MEAS_CLOCKSTR_L = 0x2C10, ///< clock stretching, low repeatability
  SHT3X_MEAS_POLLING_H = 0x2400,  ///< polling, high repeatability
  SHT3X_MEAS_POLLING_M = 0x240B,  ///< polling, medium repeatability
  SHT3X_MEAS_POLLING_L = 0x2416,  ///< polling, low repeatability
  SHT3X_MEAS_PERI_05_H = 0x2032,  ///< periodic 0.5 mps, high repeatability
  SHT3X_MEAS_PERI_05_M = 0x2024,  ///< periodic 0.5 mps, medium repeatability
  SHT3X_MEAS_PERI_05_L = 0x202F,  ///< periodic 0.5 mps, low repeatability
  SHT3X_MEAS_PERI_1_H = 0x2130,   ///< periodic 1 mps, high repeatability
  SHT3X_MEAS_PERI_1_M = 0x2126,   ///< periodic 1 mps, medium repeatability
  SHT3X_MEAS_PERI_1_L = 0x212D,   ///< periodic 1 mps, low repeatability
  SHT3X_MEAS_PERI_2_H = 0x2236,   ///< periodic 2 mps, high repeatability
  SHT3X_MEAS_PERI_2_M = 0x2220,   ///< periodic 2 mps, medium repeatability
  SHT3X_MEAS_PERI_2_L = 0x222B,   ///< periodic 2 mps, low repeatability
  SHT3X_MEAS_PERI_4_H = 0x2334,   ///< periodic 4 mps, high repeatability
  SHT3X_MEAS_PERI_4_M = 0x2322,   ///< periodic 4 mps, medium repeatability
  SHT3X_MEAS_PERI_4_L = 0x2329,   ///< periodic 4 mps, low repeatability
  SHT3X_MEAS_PERI_10_H = 0x2737,  ///< periodic 10 mps, high repeatability
  SHT3X_MEAS_PERI_10_M = 0x2721,  ///< periodic 10 mps, medium repeatability
  SHT3X_MEAS_PERI_10_L = 0x272A,  ///< periodic 10 mps, low repeatability
  SHT3X_FETCH_DATA = 0xE000,      ///< readout measurements for periodic mode
  SHT3X_R_AL_LIM_LS = 0xE102,     ///< read alert limits, low set
  SHT3X_R_AL_LIM_LC = 0xE109,     ///< read alert limits, low clear
  SHT3X_R_AL_LIM_HS = 0xE11F,     ///< read alert limits, high set
  SHT3X_R_AL_LIM_HC = 0xE114,     ///< read alert limits, high clear
  SHT3X_W_AL_LIM_HS = 0x611D,     ///< write alert limits, high set
  SHT3X_W_AL_LIM_HC = 0x6116,     ///< write alert limits, high clear
  SHT3X_W_AL_LIM_LC = 0x610B,     ///< write alert limits, low clear
  SHT3X_W_AL_LIM_LS = 0x6100,     ///< write alert limits, low set
  SHT3X_NO_SLEEP = 0x303E,
} sht3xCommands;

/// Measurement Repeatability
typedef enum sht3xRepeatability {
  REPEATAB_HIGH,   ///< high repeatability
  REPEATAB_MEDIUM, ///< medium repeatability
  REPEATAB_LOW,    ///< low repeatability
} sht3xRepeatability;

/// Measurement Mode
typedef enum sht3xMode {
  MODE_CLKSTRETCH, ///< clock stretching
  MODE_POLLING,    ///< polling
} sht3xMode;

/// Measurement frequency
typedef enum sht3xFrequency {
  FREQUENCY_HZ5,  ///<  0.5 measurements per seconds
  FREQUENCY_1HZ,  ///<  1.0 measurements per seconds
  FREQUENCY_2HZ,  ///<  2.0 measurements per seconds
  FREQUENCY_4HZ,  ///<  4.0 measurements per seconds
  FREQUENCY_10HZ, ///< 10.0 measurements per seconds
} sht3xFrequency;

/// Error codes
typedef enum sht3xError {
  NO_ERROR = 0x00,       ///< no error
  ACK_ERROR = 0x01,      ///< no acknowledgment error
  CHECKSUM_ERROR = 0x02, ///< checksum mismatch error
  TIMEOUT_ERROR = 0x04,  ///< timeout error
  PARM_ERROR = 0x80,     ///< parameter out of range error
} sht3xError;

//-- public class -------------------------------------------------------------

/**
 * \brief	Sensirion SHT3x Temperature and Humidity Sensor
 */
class Sht3x {
public: ///< begin of the public part of the class
  /**
   * \brief	Constructs a sensor instance.
   *
   * \param _i2c    A pointer to an i2c object
   * \param _addr   I2C address
   */
  Sht3x(I2C *_i2c, uint8_t _addr = SHT3X_ADDR);

  ~Sht3x();

  /**
   * \brief	  Sets the I2C address.
   * \param   _addr I2C address
   */
  void SetAddr(uint8_t _addr);

  /**
   * \brief   Reads the serial number of the sensor.
   * \param   serialNumber pointer to the serial number
   * \return  error code
   */
  sht3xError ReadSerialNumber(uint32_t *serialNumber);

  /**
   * \brief   Reads the status register from the sensor.
   * \param   status pointer to the serial number
   * \return  error code
   */
  sht3xError ReadStatus(uint16_t *status);

  /**
   * \brief     Clears all alers Flags in the status register of the sensor.
   * \return    error code
   */
  sht3xError ClearAllAlertFlags(void);

  /**
   * \brief     Gets the temperature [degC] and relative humidity [%RH].
   * \param     temperature pointer to the temperature
   * \param     humidity pointer to the humidity
   * \param     repeatability repeatability for the measurement [low, medium,
   *                          high]
   * \param     mode command mode [clock stretching, polling]
   * \param     timeout timeout in milliseconds (only for polling)
   * \return    error code
   */
  sht3xError GetTempAndHumi(float *temperature, float *humidity,
                            sht3xRepeatability repeatability, sht3xMode mode,
                            uint8_t timeout = 255);

  /**
   * \brief Starts periodic measurement.
   * \param repeatability repeatability for the measurement [low, medium,
   *                      high]
   * \param frequency measurement frequency [0.5, 1, 2, 4, 10] Hz
   * \return error code
   * \warning not implemented
   */
  sht3xError StartPeriodicMeasurement(sht3xRepeatability repeatability,
                                      sht3xFrequency frequency);

  /**
   * \brief Reads last measurement from the sensor buffer.
   * \param temperature pointer to temperature
   * \param humidity pointer to humidity
   * \return error code
   * \warning not implemented
   */
  sht3xError ReadMeasurementBuffer(float *temperature, float *humidity);

  /**
   * \brief Enables the heater on the sensor.
   * \return error code
   * \warning not implemented
   */
  sht3xError EnableHeater(void);

  /**
   * \brief Diables the heater on the sensor.
   * \return error code
   * \warning not implemented
   */
  sht3xError DisableHeater(void);

  /**
   * \brief Sets the sensor alert limits.
   * \warning not implemented.
   */
  void SetAlertLimits(void);

  /**
   * \brief Gets the sensor alert limits.
   * \warning not implemented.
   */
  void GetAlertLimits(void);

  /**
   * \brief Returns the state of the alert pin.
   * \warning not implemented
   */
  void ReadAlert(void);

  /**
   * \brief Calls the soft reset mechanism that forces the sensor into
   * well-defined state without removing the power supply.
   * \return error code
   * \warning not implemented
   */
  sht3xError SoftReset(void);

  /**
   * \brief Resets the sensor by pulling down the reset pin.
   * \warning not implemented
   */
  void HardReset(void);

private:
  /// I2C acknowledge
  typedef enum I2cAck {
    ACK = 0,
    NACK = 1,
  } I2cAck;

  /// I2C direction
  typedef enum I2cDir {
    WRITE = 0,
    READ = 1,
  } I2cDir;

  uint8_t addr;  ///< i2c address
  uint8_t addr8; ///< shifted version of the address, used for the I2C Library
  I2C *i2c;      ///< pointer to i2c instance

  /**
   * \brief   Gets the temperature [degC] and relative humidity [%RH]. This
   * function uses clock stretching for waiting until the measurement is ready.
   * \param   temperature pointer to the temperature
   * \param   humidity pointer to the humidity
   * \param   repeatability repeatability for the measurement [low, medium,
   *                        high]
   * \return  error code
   */
  sht3xError GetTempAndHumiClkStretch(float *temperature, float *humidity,
                                      sht3xRepeatability repeatability);

  /**
   * \brief   Gets the temperature [degC] and relative humidity [%RH]. This
   * function polls every 1ms until the measurement is ready.
   * \param   temperature pointer to the temperature
   * \param   humidity pointer to the humidity
   * \param   repeatability repeatability for the measurement [low, medium,
   *                        high]
   * \param   timeout polling timeout in milliseconds
   * \return  error code
   * \warning not implemented.
   */
  sht3xError GetTempAndHumiPolling(float *temperature, float *humidity,
                                   sht3xRepeatability repeatability,
                                   uint8_t timeout);

  /**
   * \brief Writes command to the sensor
   * \param command command
   * \return error code
   */
  sht3xError WriteCommand(sht3xCommands command);

  /**
   * \brief Reads multible times two data bytes and one CRC byte and checks
   * the CRC. After every 16bit value the sensor sends to the bus, it
   * adds one byte checksum, before the next 16bit are sent. Since not every
   * command returns the dame amount od data, the function is implemented like
   * this.
   * \param data pointer to the data array
   * \param n how many times the three bytes should be read
   * \return error code
   */
  sht3xError ReadBytesAndCrc(uint16_t *data, uint8_t n=1);

  /**
   * \brief Checks the checksum.
   * \param data pointer to data to be checked
   * \param len num of data bytes
   * \param checksum checksum byte
   */
  sht3xError CheckCrc(char *data, uint8_t len, char checksum);

  /**
   * \brief Calculates the temperature in degrees celsius from the raw sensor
   * data.
   * \param rawValue raw temperature sensor value
   * \return temperature in degree celsius
   */
  float CalcTemperature(uint16_t rawValue);

  /**
   * \brief Calculates the relative humidity from the raw sensor data.
   * \param rawValue raw temperature sensor value
   * \return relative humidity in percent
   */
  float CalcHumidity(uint16_t rawValue);

  /**
   * \brief converts the Mbed I2C error code to the SHT error code
   * \param i2cErr i2c error code
   * \param dir of the last transmission
   */
  sht3xError ConvertI2cError(uint8_t i2cErr, I2cDir dir);
};

#endif /* _SHT3X_H */