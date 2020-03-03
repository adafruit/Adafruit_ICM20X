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
#define ICM20948_I2CADDR_DEFAULT 0x69 ///< ICM20948 default i2c address

// Bank 0
#define ICM20948_I2C_MST_STATUS 0x17
// Bank 3
#define ICM20948_I2C_MST_ODR_CONFIG 0x0
#define ICM20948_I2C_MST_CTRL  0x1
#define ICM20948_I2C_MST_DELAY_CTRL  0x2
#define ICM20948_I2C_SLV0_ADDR 0x3
#define ICM20948_I2C_SLV0_REG  0x4
#define ICM20948_I2C_SLV0_CTRL 0x5
#define ICM20948_I2C_SLV0_DO 0x6

#define ICM20948_I2C_SLV4_ADDR 0x13
#define ICM20948_I2C_SLV4_REG  0x14
#define ICM20948_I2C_SLV4_CTRL 0x15
#define ICM20948_I2C_SLV4_DO 0x16
#define ICM20948_I2C_SLV4_DI 0x17


/** The accelerometer data range */
typedef enum {
  ICM20948_ACCEL_RANGE_2_G,
  ICM20948_ACCEL_RANGE_4_G,
  ICM20948_ACCEL_RANGE_8_G,
  ICM20948_ACCEL_RANGE_16_G,
} icm20948_accel_range_t;

/** The gyro data range */
typedef enum {
  ICM20948_GYRO_RANGE_250_DPS,
  ICM20948_GYRO_RANGE_500_DPS,
  ICM20948_GYRO_RANGE_1000_DPS,
  ICM20948_GYRO_RANGE_2000_DPS,
} icm20948_gyro_range_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ST ICM2948 9-DoF Accelerometer, gyro, and magnetometer
 */
class Adafruit_ICM20948 : public Adafruit_ICM20X {
public:
  Adafruit_ICM20948();
  ~Adafruit_ICM20948(){};
  bool begin_I2C(uint8_t i2c_addr = ICM20948_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);
  uint8_t _read_ext_reg(uint8_t slv_addr, uint8_t reg_addr) ;

  bool _setupMag(void);

  icm20948_accel_range_t getAccelRange(void);
  void setAccelRange(icm20948_accel_range_t new_accel_range);

  icm20948_gyro_range_t getGyroRange(void);
  void setGyroRange(icm20948_gyro_range_t new_gyro_range);

private:
  void _scale_values(void);
};

#endif