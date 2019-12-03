/*!
 *  @file Adafruit_ICM20649.h
 *
 * 	I2C Driver for the Adafruit ICM20649 6-DoF Wide-Range Accelerometer and Gyro library
 *
 * 	This is a library for the Adafruit ICM20649 breakout:
 * 	https://www.adafruit.com/products/XXX
 * 
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_ICM20649_H
#define _ADAFRUIT_ICM20649_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>



#define ICM20649_I2CADDR_DEFAULT 0x68 ///< ICM20649 default i2c address
#define ICM20649_CHIP_ID 0xE1 ///< ICM20649 default device id from WHOAMI

// Bank 0
#define ICM20649_WHOAMI 0x00 ///< Chip ID register
#define ICM20649_REG_BANK_SEL 0x7F ///<  # register bank selection register
#define ICM20649_PWR_MGMT_1  0x06 ///<  #primary power management register
#define ICM20649_ACCEL_XOUT_H 0x2D ///<  # first byte of accel data
#define ICM20649_GYRO_XOUT_H 0x33 ///<  # first byte of accel data

// Bank 2
#define ICM20649_GYRO_SMPLRT_DIV 0x00 ///< 
#define ICM20649_GYRO_CONFIG_1     0x01 ///< 
#define ICM20649_ACCEL_SMPLRT_DIV_1 0x10 ///< 
#define ICM20649_ACCEL_SMPLRT_DIV_2 0x11 ///< 
#define ICM20649_ACCEL_CONFIG_1     0x14 ///< 

#define G_TO_ACCEL            9.80665

/**
 * @brief Example enum values
 *
 * Allowed values for `setProximityLEDCurrent`.
 */
/** The accelerometer data range */
typedef enum accel_range {
  ICM20649_ACCEL_RANGE_4_G,
  ICM20649_ACCEL_RANGE_8_G,
  ICM20649_ACCEL_RANGE_16_G,
  ICM20649_ACCEL_RANGE_30_G,
} icm20649_accel_range_t;

/** The gyro data range */
typedef enum gyro_range {
  ICM20649_GYRO_RANGE_500_DPS,
  ICM20649_GYRO_RANGE_1000_DPS,
  ICM20649_GYRO_RANGE_2000_DPS,
  ICM20649_GYRO_RANGE_4000_DPS,
} icm20649_gyro_range_t;


/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ICM20649 I2C Digital Potentiometer
 */
class Adafruit_ICM20649 {
public:

  Adafruit_ICM20649();  
  bool begin(uint8_t i2c_address=ICM20649_I2CADDR_DEFAULT, TwoWire *wire = &Wire, int32_t sensorID = 0);

  icm20649_accel_range_t getAccelRange(void);
  void setAccelRange(icm20649_accel_range_t new_accel_range);

  icm20649_gyro_range_t getGyroRange(void);
  void setGyroRange(icm20649_gyro_range_t new_gyro_range);
  
  void reset(void);
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp);

private:
  void _read(void);

  bool _init(void);
  
  void _setBank(uint8_t bank_number);
  float temperature, accX, accY, accZ, gyroX, gyroY, gyroZ;
  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

  uint8_t _sensorid_accel, _sensorid_gyro, _sensorid_temp;
  
  Adafruit_I2CDevice *i2c_dev;
  Adafruit_SPIDevice *spi_dev;
};

#endif

