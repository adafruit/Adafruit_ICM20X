/*!
 *  @file Adafruit_ICM20X.h
 *
 * 	I2C Driver for the Adafruit ICM20X 6-DoF Wide-Range Accelerometer and
 *Gyro library
 *
 * 	This is a library for the Adafruit ICM20X breakout:
 * 	https://www.adafruit.com/products/4464
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_ICM20X_H
#define _ADAFRUIT_ICM20X_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Bank 0
#define ICM20X_WHOAMI 0x00           ///< Chip ID register
#define ICM20X_USER_CTRL 0x03        ///< Chip ID register
#define ICM20X_LP_CONFIG 0x05        ///< Chip ID register
#define ICM20X_REG_INT_PIN_CFG 0xF   ///< Interrupt config register
#define ICM20X_REG_INT_ENABLE 0x10   ///< Interrupt enable register 0
#define ICM20X_REG_INT_ENABLE_1 0x11 ///< Interrupt enable register 1
#define ICM20X_REG_BANK_SEL 0x7F     ///< register bank selection register
#define ICM20X_PWR_MGMT_1 0x06       ///< primary power management register
#define ICM20X_ACCEL_XOUT_H 0x2D     ///< first byte of accel data
#define ICM20X_GYRO_XOUT_H 0x33      ///< first byte of accel data

// Bank 2
#define ICM20X_GYRO_SMPLRT_DIV 0x00    ///< Gyroscope data rate divisor
#define ICM20X_GYRO_CONFIG_1 0x01      ///< Gyro config for range setting
#define ICM20X_ACCEL_SMPLRT_DIV_1 0x10 ///< Accel data rate divisor MSByte
#define ICM20X_ACCEL_SMPLRT_DIV_2 0x11 ///< Accel data rate divisor LSByte
#define ICM20X_ACCEL_CONFIG_1 0x14     ///< Accel config for setting range

#define ICM20948_CHIP_ID 0xEA ///< ICM20948 default device id from WHOAMI
#define ICM20649_CHIP_ID 0xE1 ///< ICM20649 default device id from WHOAMI

/////////////////////////////////////////

class Adafruit_ICM20X;

/** Adafruit Unified Sensor interface for accelerometer component of ICM20X */
class Adafruit_ICM20X_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Accelerometer(Adafruit_ICM20X *parent) {
    _theICM20X = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20A;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of ICM20X */
class Adafruit_ICM20X_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Gyro(Adafruit_ICM20X *parent) { _theICM20X = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20B;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/** Adafruit Unified Sensor interface for magnetometer component of ICM20X */
class Adafruit_ICM20X_Magnetometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the magnetometer
     sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Magnetometer(Adafruit_ICM20X *parent) { _theICM20X = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20C;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/** Adafruit Unified Sensor interface for temperature component of ICM20X */
class Adafruit_ICM20X_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Temp(Adafruit_ICM20X *parent) { _theICM20X = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20D;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ST ICM20X 6-DoF Accelerometer and Gyro
 */
class Adafruit_ICM20X {
public:
  Adafruit_ICM20X();
  ~Adafruit_ICM20X();

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensor_id = 0);

  uint8_t getGyroRateDivisor(void);
  void setGyroRateDivisor(uint8_t new_gyro_divisor);

  uint16_t getAccelRateDivisor(void);
  void setAccelRateDivisor(uint16_t new_accel_divisor);

  void reset(void);
  void setInt1ActiveLow(bool active_low);
  void setInt2ActiveLow(bool active_low);
  void setI2CBypass(bool bypass_i2c);

  Adafruit_Sensor *getAccelerometerSensor(void);
  Adafruit_Sensor *getGyroSensor(void);
  Adafruit_Sensor *getMagnetometerSensor(void);
  Adafruit_Sensor *getTemperatureSensor(void);

  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                sensors_event_t *temp, sensors_event_t *mag = NULL);

protected:
  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ,         ///< Last reading's gyro Z axis in rad/s
      magX,          ///< Last reading's mag X axis in rad/s
      magY,          ///< Last reading's mag Y axis in rad/s
      magZ;          ///< Last reading's mag Z axis in rad/s

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  Adafruit_ICM20X_Accelerometer *accel_sensor =
      NULL;                                 ///< Accelerometer data object
  Adafruit_ICM20X_Gyro *gyro_sensor = NULL; ///< Gyro data object
  Adafruit_ICM20X_Magnetometer *mag_sensor =
      NULL;                                 ///< Magnetometer sensor data object
  Adafruit_ICM20X_Temp *temp_sensor = NULL; ///< Temp sensor data object
  uint16_t _sensorid_accel,                 ///< ID number for accelerometer
      _sensorid_gyro,                       ///< ID number for gyro
      _sensorid_mag,                        ///< ID number for mag
      _sensorid_temp;                       ///< ID number for temperature

  void _read(void);
  virtual void _scale_values(void);
  virtual bool begin_I2C(uint8_t i2c_add, TwoWire *wire, int32_t sensor_id);
  // virtual bool _init(int32_t sensor_id);
  bool _init(int32_t sensor_id);
  int16_t rawAccX, ///< temp variables
      rawAccY,     ///< temp variables
      rawAccZ,     ///< temp variables
      rawTemp,     ///< temp variables
      rawGyroX,    ///< temp variables
      rawGyroY,    ///< temp variables
      rawGyroZ,    ///< temp variables
      rawMagX,     ///< temp variables
      rawMagY,     ///< temp variables
      rawMagZ;     ///< temp variables

  // virtual void _setBank(uint8_t bank_number);
  void _setBank(uint8_t bank_number);

  uint8_t readAccelRange(void);
  void writeAccelRange(uint8_t new_accel_range);

  uint8_t readGyroRange(void);
  void writeGyroRange(uint8_t new_gyro_range);

private:
  void init1(void);
  friend class Adafruit_ICM20X_Accelerometer; ///< Gives access to private
                                              ///< members to Accelerometer
                                              ///< data object
  friend class Adafruit_ICM20X_Gyro; ///< Gives access to private members to
                                     ///< Gyro data object
  friend class Adafruit_ICM20X_Magnetometer; ///< Gives access to private
                                             ///< members to Magnetometer data
                                             ///< object

  friend class Adafruit_ICM20X_Temp; ///< Gives access to private members to
                                     ///< Temp data object

  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
  void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillMagEvent(sensors_event_t *mag, uint32_t timestamp);
};

#endif
