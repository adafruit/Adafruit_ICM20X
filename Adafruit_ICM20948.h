/*!
 *  @file Adafruit_ICM20948.h
 *
 * 	I2C Driver for the Adafruit ICM20948 9-DoF Accelerometer, Gyro, and
 *Magnetometer library
 *
 * 	This is a library for the Adafruit ICM20948 breakout:
 * 	https://www.adafruit.com/products/4544
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_ICM20948_H
#define _ADAFRUIT_ICM20948_H

#include "Adafruit_ICM20X.h"

// /** The accelerometer data range */
// typedef enum {
//   ICM20948_ACCEL_RANGE_2_G,
//   ICM20948_ACCEL_RANGE_4_G,
//   ICM20948_ACCEL_RANGE_8_G,
//   ICM20948_ACCEL_RANGE_16_G,
// } icm20948_accel_range_t;

// /** The gyro data range */
// typedef enum {
//   ICM20948_GYRO_RANGE_250_DPS,
//   ICM20948_GYRO_RANGE_500_DPS,
//   ICM20948_GYRO_RANGE_1000_DPS,
//   ICM20948_GYRO_RANGE_2000_DPS,
// } icm20948_gyro_range_t;


// class Adafruit_ISM330DHCT : public Adafruit_LSM6DSOX {
// class Adafruit_LSM6DSOX : public Adafruit_LSM6DS {


class Adafruit_ICM20948 : public Adafruit_ICM20X {
public:
  Adafruit_ICM20948();
  ~Adafruit_ICM20948();

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
  float temperature,                  ///< Last reading's temperature (C)
      accX,  ///< Last reading's accelerometer X axis m/s^2
      accY,  ///< Last reading's accelerometer Y axis m/s^2
      accZ,  ///< Last reading's accelerometer Z axis m/s^2
      gyroX, ///< Last reading's gyro X axis in rad/s
      gyroY, ///< Last reading's gyro Y axis in rad/s
      gyroZ; ///< Last reading's gyro Z axis in rad/s
  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
  icm20x_accel_range_t getAccelRange(void);
  icm20x_gyro_range_t getGyroRange(void);

protected:
  virtual bool _init(int32_t sensor_id);

private:
  void _read(void);
  void _setBank(uint8_t bank_number);
};

#endif