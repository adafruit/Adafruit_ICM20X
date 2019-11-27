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
/**
 * @brief Example enum values
 *
 * Allowed values for `setProximityLEDCurrent`.
 */
typedef enum led_current {
  ICM20649_EXAMPLE_50MA,
  ICM20649_EXAMPLE_75MA,
} ICM20649_example_t;


/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ICM20649 I2C Digital Potentiometer
 */
class Adafruit_ICM20649 {
public:

  Adafruit_ICM20649();  
  bool begin(uint8_t i2c_address=ICM20649_I2CADDR_DEFAULT, TwoWire *wire = &Wire);

  ICM20649_example_t getEXAMPLE(void);
  void setEXAMPLE(ICM20649_example_t example_value);

private:
  bool _init(void);

  Adafruit_I2CDevice *i2c_dev;
};

#endif

