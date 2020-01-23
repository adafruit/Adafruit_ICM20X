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

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define ICM20649_I2CADDR_DEFAULT 0x68 ///< ICM20649 default i2c address
#define ICM20649_CHIP_ID 0xE1 ///< ICM20649 default device id from WHOAMI

// Bank 0
#define ICM20649_WHOAMI 0x00           ///< Chip ID register
#define ICM20649_REG_INT_PIN_CFG 0xF   ///< Interrupt config register
#define ICM20649_REG_INT_ENABLE 0x10   ///< Interrupt enable register 0
#define ICM20649_REG_INT_ENABLE_1 0x11 ///< Interrupt enable register 1
#define ICM20649_REG_BANK_SEL 0x7F     ///< register bank selection register
#define ICM20649_PWR_MGMT_1 0x06       ///< primary power management register
#define ICM20649_ACCEL_XOUT_H 0x2D     ///< first byte of accel data
#define ICM20649_GYRO_XOUT_H 0x33      ///< first byte of accel data

// Bank 2
#define ICM20649_GYRO_SMPLRT_DIV 0x00    ///< Gyroscope data rate divisor
#define ICM20649_GYRO_CONFIG_1 0x01      ///< Gyro config for range setting
#define ICM20649_ACCEL_SMPLRT_DIV_1 0x10 ///< Accel data rate divisor MSByte
#define ICM20649_ACCEL_SMPLRT_DIV_2 0x11 ///< Accel data rate divisor LSByte
#define ICM20649_ACCEL_CONFIG_1 0x14     ///< Accel config for setting range

/**
 * @brief Example enum values
 *
 * Allowed values for `setProximityLEDCurrent`.
 */
/** The accelerometer data range */
typedef enum {
  ICM20649_ACCEL_RANGE_4_G,
  ICM20649_ACCEL_RANGE_8_G,
  ICM20649_ACCEL_RANGE_16_G,
  ICM20649_ACCEL_RANGE_30_G,
} icm20649_accel_range_t;

/** The gyro data range */
typedef enum {
  ICM20649_GYRO_RANGE_500_DPS,
  ICM20649_GYRO_RANGE_1000_DPS,
  ICM20649_GYRO_RANGE_2000_DPS,
  ICM20649_GYRO_RANGE_4000_DPS,
} icm20649_gyro_range_t;

class Adafruit_ICM20649;

/** Adafruit Unified Sensor interface for temperature component of ICM20649 */
class Adafruit_ICM20649_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the ICM20649 class */
  Adafruit_ICM20649_Temp(Adafruit_ICM20649 *parent) { _theICM20649 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x649;
  Adafruit_ICM20649 *_theICM20649 = NULL;
};

/** Adafruit Unified Sensor interface for accelerometer component of ICM20649 */
class Adafruit_ICM20649_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the ICM20649 class */
  Adafruit_ICM20649_Accelerometer(Adafruit_ICM20649 *parent) {
    _theICM20649 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x64A;
  Adafruit_ICM20649 *_theICM20649 = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of ICM20649 */
class Adafruit_ICM20649_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the ICM20649 class */
  Adafruit_ICM20649_Gyro(Adafruit_ICM20649 *parent) { _theICM20649 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x64B;
  Adafruit_ICM20649 *_theICM20649 = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ST ICM20649 6-DoF Accelerometer and Gyro
 */
class Adafruit_ICM20649 {
public:
  Adafruit_ICM20649();
  ~Adafruit_ICM20649();

  bool begin_I2C(uint8_t i2c_addr = ICM20649_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensor_id = 0);

  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                sensors_event_t *temp);

  icm20649_accel_range_t getAccelRange(void);
  void setAccelRange(icm20649_accel_range_t new_accel_range);

  icm20649_gyro_range_t getGyroRange(void);
  void setGyroRange(icm20649_gyro_range_t new_gyro_range);

  uint8_t getGyroRateDivisor(void);
  void setGyroRateDivisor(uint8_t new_gyro_divisor);

  uint16_t getAccelRateDivisor(void);
  void setAccelRateDivisor(uint16_t new_accel_divisor);

  void reset(void);
  void setInt1ActiveLow(bool active_low);
  void setInt2ActiveLow(bool active_low);
  void setI2CBypass(bool bypass_i2c);

  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getAccelerometerSensor(void);
  Adafruit_Sensor *getGyroSensor(void);

protected:
  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ;         ///< Last reading's gyro Z axis in rad/s

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  Adafruit_ICM20649_Temp *temp_sensor = NULL; ///< Temp sensor data object
  Adafruit_ICM20649_Accelerometer *accel_sensor =
      NULL;                                   ///< Accelerometer data object
  Adafruit_ICM20649_Gyro *gyro_sensor = NULL; ///< Gyro data object

  uint16_t _sensorid_accel, ///< ID number for accelerometer
      _sensorid_gyro,       ///< ID number for gyro
      _sensorid_temp;       ///< ID number for temperature

  void _read(void);
  virtual bool _init(int32_t sensor_id);

private:
  friend class Adafruit_ICM20649_Temp; ///< Gives access to private members to
                                       ///< Temp data object
  friend class Adafruit_ICM20649_Accelerometer; ///< Gives access to private
                                                ///< members to Accelerometer
                                                ///< data object
  friend class Adafruit_ICM20649_Gyro; ///< Gives access to private members to
                                       ///< Gyro data object

  void _setBank(uint8_t bank_number);
  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
  void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
};

#endif
