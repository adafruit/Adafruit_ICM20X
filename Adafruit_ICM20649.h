/*!
 *  @file Adafruit_ICM20649.h
 *
 * 	I2C Driver for the Adafruit ICM20649 6-DoF Wide-Range Accelerometer and
 *Gyro library
 *
 * 	This is a library for the Adafruit ICM20649 breakout:
 * 	https://www.adafruit.com/products/4464
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

#include "Adafruit_ICM20X.h"

#define ICM20649_I2CADDR_DEFAULT 0x68 ///< ICM20649 default i2c address
#define ICM20649_CHIP_ID 0xE1 ///< ICM20649 default device id from WHOAMI


class Adafruit_ICM20649 : public Adafruit_ICM20X {
public:
  Adafruit_ICM20649();
  ~Adafruit_ICM20649(){};
  // Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  // Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
  // float temperature,                  ///< Last reading's temperature (C)
  //     accX,  ///< Last reading's accelerometer X axis m/s^2
  //     accY,  ///< Last reading's accelerometer Y axis m/s^2
  //     accZ,  ///< Last reading's accelerometer Z axis m/s^2
  //     gyroX, ///< Last reading's gyro X axis in rad/s
  //     gyroY, ///< Last reading's gyro Y axis in rad/s
  //     gyroZ; ///< Last reading's gyro Z axis in rad/s
  // int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
  // icm20x_accel_range_t getAccelRange(void);
  // icm20x_gyro_range_t getGyroRange(void);

private:
  // void _read(void);
  // void _setBank(uint8_t bank_number);
};

#endif
