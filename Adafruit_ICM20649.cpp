
/*!
 *  @file Adafruit_ICM20649.cpp
 *
 *  @mainpage Adafruit ICM20649 6-DoF Wide-Range Accelerometer and Gyro library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit ICM20649 6-DoF Wide-Range Accelerometer and
 * Gyro library
 *
 * 	This is a library for the Adafruit ICM20649 breakout:
 * 	https://www.adafruit.com/product/4464
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_ICM20649.h"

/*!
 *    @brief  Instantiates a new ICM20649 class
 */
Adafruit_ICM20649::Adafruit_ICM20649(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            An optional parameter to set the sensor ids to differentiate
 * similar sensors The passed value is assigned to the accelerometer and the
 * gyro get +1 and the temperature sensor +2.
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_ICM20649::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                                 int32_t sensor_id) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    Serial.println("I2C begin Failed");
    return false;
  }

  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;
  return _init();
}


/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_ICM20649::begin_SPI(uint8_t cs_pin, SPIClass *theSPI, int32_t sensor_id) {
  i2c_dev = NULL;

  spi_dev = new Adafruit_SPIDevice(cs_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0,             // data mode
                                   theSPI);

  if (!spi_dev->begin()) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;
  return _init();
  // return true;
}

/*!
 *    @brief  Sets up the hardware and initializes software SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  sck_pin The arduino pin # connected to SPI clock
 *    @param  miso_pin The arduino pin # connected to SPI MISO
 *    @param  mosi_pin The arduino pin # connected to SPI MOSI
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_ICM20649::begin_SPI(int8_t cs_pin, int8_t sck_pin,
                                  int8_t miso_pin, int8_t mosi_pin, int32_t sensor_id) {
  i2c_dev = NULL;

  spi_dev = new Adafruit_SPIDevice(cs_pin, sck_pin, miso_pin, mosi_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0);            // data mode
  if (!spi_dev->begin()) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;
  return _init();
  // return true;
}

/**
 * @brief Reset the internal registers and restores the default settings
 *
 */
void Adafruit_ICM20649::reset(void) {
  _setBank(0);

  Adafruit_BusIO_Register pwr_mgmt1 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20649_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits reset_bit =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt1, 1, 7);

  reset_bit.write(1);
  delay(20);

  while (reset_bit.read()) {
    delay(10);
  };
}
boolean Adafruit_ICM20649::_init(void) {

  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_WHOAMI, 2);

  Adafruit_BusIO_Register reg_bank_sel =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_REG_BANK_SEL);

  Adafruit_BusIO_RegisterBits bank =
      Adafruit_BusIO_RegisterBits(&reg_bank_sel, 2, 4);

  _setBank(0);

  // make sure we're talking to the right chip
  if (chip_id.read() != ICM20649_CHIP_ID) {
    return false;
  }
  // do any software reset or other initial setup
  reset();

  // _sleep = RWBit(_ICM20649_PWR_MGMT_1, 6)
  Adafruit_BusIO_Register pwr_mgmt_1 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_PWR_MGMT_1);

  Adafruit_BusIO_RegisterBits sleep =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt_1, 1, 6);

  Adafruit_BusIO_RegisterBits clock_source =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt_1, 3, 0);

  sleep.write(false);    // take out of default sleep state
  clock_source.write(1); // AUTO SELECT BEST CLOCK

  // set ACCEL
  _setBank(2);
  Adafruit_BusIO_Register accel_config_1 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_CONFIG_1);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&accel_config_1, 2, 1);

  accel_range.write(0x01); // SET ACCEL RANGE TO 8G
  accel_range.read();

  // self._cached_accel_range = self._accel_range

  // #TODO: CV-ify
  // self._accel_dlpf_config = 3

  // # 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]),
  // # 1125Hz/(1+20) = 53.57Hz
  // self._accel_rate_divisor = 20

  // # writeByte(ICM20649_ADDR,GYRO_CONFIG_1, gyroConfig);
  // self._gyro_range = GyroRange.RANGE_500_DPS #pylint: disable=no-member
  // sleep(0.100)
  // self._cached_gyro_range = self._gyro_range

  // # //ORD = 1100Hz/(1+10) = 100Hz
  // self._gyro_rate_divisor = 0x0A

  _setBank(0);

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  accel
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with acceleration event data.
    @param  gyro
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with gyro event data.
    @param  temp
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with temperature event data.
    @return True on successful read
*/
/**************************************************************************/
bool Adafruit_ICM20649::getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                                 sensors_event_t *temp) {
  uint32_t t = millis();
  _read();

  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = t;
  accel->acceleration.x = accX * SENSORS_GRAVITY_EARTH;
  accel->acceleration.y = accY * SENSORS_GRAVITY_EARTH;
  accel->acceleration.z = accZ * SENSORS_GRAVITY_EARTH;

  memset(gyro, 0, sizeof(sensors_event_t));
  gyro->version = 1;
  gyro->sensor_id = _sensorid_gyro;
  gyro->type = SENSOR_TYPE_GYROSCOPE;
  gyro->timestamp = t;
  gyro->gyro.x = gyroX;
  gyro->gyro.y = gyroY;
  gyro->gyro.z = gyroZ;

  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = t;
  temp->temperature = (temperature / 333.87) + 21.0;

  return true;
}

/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void Adafruit_ICM20649::_read(void) {

  _setBank(0);

  Adafruit_BusIO_Register data_reg =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_XOUT_H, 14);

  uint8_t buffer[14];
  data_reg.read(buffer, 14);

  rawAccX = buffer[0] << 8 | buffer[1];
  rawAccY = buffer[2] << 8 | buffer[3];
  rawAccZ = buffer[4] << 8 | buffer[5];

  rawGyroX = buffer[6] << 8 | buffer[7];
  rawGyroY = buffer[8] << 8 | buffer[9];
  rawGyroZ = buffer[10] << 8 | buffer[11];

  temperature = buffer[12] << 8 | buffer[13];

  icm20649_gyro_range_t gyro_range = getGyroRange();

  float gyro_scale = 1.0;

  if (gyro_range == ICM20649_GYRO_RANGE_500_DPS)
    gyro_scale = 65.5;
  if (gyro_range == ICM20649_GYRO_RANGE_1000_DPS)
    gyro_scale = 32.8;
  if (gyro_range == ICM20649_GYRO_RANGE_2000_DPS)
    gyro_scale = 16.4;
  if (gyro_range == ICM20649_GYRO_RANGE_4000_DPS)
    gyro_scale = 8.2;

  gyroX = rawGyroX / gyro_scale;
  gyroY = rawGyroY / gyro_scale;
  gyroZ = rawGyroZ / gyro_scale;

  icm20649_accel_range_t accel_range = getAccelRange();
  float accel_scale = 1.0;

  if (accel_range == ICM20649_ACCEL_RANGE_4_G)
    accel_scale = 8192.0;
  if (accel_range == ICM20649_ACCEL_RANGE_8_G)
    accel_scale = 4096.0;
  if (accel_range == ICM20649_ACCEL_RANGE_16_G)
    accel_scale = 2048.0;
  if (accel_range == ICM20649_ACCEL_RANGE_30_G)
    accel_scale = 1024.0;

  accX = rawAccX / accel_scale;
  accY = rawAccY / accel_scale;
  accZ = rawAccZ / accel_scale;
  _setBank(0);
}

/**************************************************************************/
/*!
    @brief Sets register bank.
    @param  bank_number
          The bank to set to active
*/
void Adafruit_ICM20649::_setBank(uint8_t bank_number) {

  Adafruit_BusIO_Register reg_bank_sel =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_REG_BANK_SEL);

  Adafruit_BusIO_RegisterBits bank =
      Adafruit_BusIO_RegisterBits(&reg_bank_sel, 2, 4);

  bank.write(bank_number);
}

/**************************************************************************/
/*!
    @brief Get the accelerometer's measurement range.
    @returns The accelerometer's measurement range (`icm20649_accel_range_t`).
*/
icm20649_accel_range_t Adafruit_ICM20649::getAccelRange(void) {
  _setBank(2);

  Adafruit_BusIO_Register accel_config_1 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_CONFIG_1);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&accel_config_1, 2, 1);

  icm20649_accel_range_t range_val = (icm20649_accel_range_t)accel_range.read();

  _setBank(0);

  return range_val;
}

/**************************************************************************/
/*!

    @brief Sets the accelerometer's measurement range.
    @param  new_accel_range
            Measurement range to be set. Must be an
            `icm20649_accel_range_t`.
*/
void Adafruit_ICM20649::setAccelRange(icm20649_accel_range_t new_accel_range) {
  _setBank(2);

  Adafruit_BusIO_Register accel_config_1 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_CONFIG_1);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&accel_config_1, 2, 1);

  accel_range.write(new_accel_range);
  _setBank(0);
}

/**************************************************************************/
/*!
    @brief Get the gyro's measurement range.
    @returns The gyro's measurement range (`icm20649_gyro_range_t`).
*/
icm20649_gyro_range_t Adafruit_ICM20649::getGyroRange(void) {
  _setBank(2);

  Adafruit_BusIO_Register gyro_config_1 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_GYRO_CONFIG_1);

  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&gyro_config_1, 2, 1);

  icm20649_gyro_range_t range_val = (icm20649_gyro_range_t)gyro_range.read();

  _setBank(0);
  return range_val;
}

/**************************************************************************/
/*!

    @brief Sets the gyro's measurement range.
    @param  new_gyro_range
            Measurement range to be set. Must be an
            `icm20649_gyro_range_t`.
*/
void Adafruit_ICM20649::setGyroRange(icm20649_gyro_range_t new_gyro_range) {
  _setBank(2);

  Adafruit_BusIO_Register gyro_config_1 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_GYRO_CONFIG_1);

  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&gyro_config_1, 2, 1);

  gyro_range.write(new_gyro_range);
  _setBank(0);
}

/**************************************************************************/
/*!
    @brief Get the accelerometer's data rate divisor.
    @returns The accelerometer's data rate divisor (`uint8_t`).
*/
uint16_t Adafruit_ICM20649::getAccelRateDivisor(void) {
  _setBank(2);

  Adafruit_BusIO_Register accel_rate_divisor =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_SMPLRT_DIV_1, 2);

  uint16_t divisor_val = accel_rate_divisor.read();

  _setBank(0);
  return divisor_val;
}

/**************************************************************************/
/*!

    @brief Sets the accelerometer's data rate divisor.
    @param  new_accel_divisor
            The accelerometer's data rate divisor (`uint16_t`). This 12-bit
   value must be <= 4095
*/
void Adafruit_ICM20649::setAccelRateDivisor(uint16_t new_accel_divisor) {
  _setBank(2);

  Adafruit_BusIO_Register accel_rate_divisor =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_SMPLRT_DIV_1, 2);

  accel_rate_divisor.write(new_accel_divisor);
  _setBank(0);
}

/**************************************************************************/
/*!
    @brief Get the gyro's data rate divisor.
    @returns The gyro's data rate divisor (`uint8_t`).
*/
uint8_t Adafruit_ICM20649::getGyroRateDivisor(void) {
  _setBank(2);

  Adafruit_BusIO_Register gyro_rate_divisor =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_GYRO_SMPLRT_DIV, 1);

  uint8_t divisor_val = gyro_rate_divisor.read();

  _setBank(0);
  return divisor_val;
}

/**************************************************************************/
/*!

    @brief Sets the gyro's data rate divisor.
    @param  new_gyro_divisor
            The gyro's data rate divisor (`uint8_t`).
*/
void Adafruit_ICM20649::setGyroRateDivisor(uint8_t new_gyro_divisor) {
  _setBank(2);

  Adafruit_BusIO_Register gyro_rate_divisor =
      Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_GYRO_SMPLRT_DIV, 1);

  gyro_rate_divisor.write(new_gyro_divisor);
  _setBank(0);
}

/*
  // _accel_dlpf_enable = RWBits(1, _ICM20649_ACCEL_CONFIG_1, 0)
  Adafruit_BusIO_Register accel_config_1 =
    Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_CONFIG_1);

  Adafruit_BusIO_RegisterBits accel_dlpf_enable =
    Adafruit_BusIO_RegisterBits(&accel_config_1, 1, 0);

  accel_dlpf_enable.write("XXX");
  accel_dlpf_enable.read();


  // _accel_dlpf_config = RWBits(3, _ICM20649_ACCEL_CONFIG_1, 3)
  Adafruit_BusIO_Register accel_config_1 =
    Adafruit_BusIO_Register(i2c_dev, spi_dev,  ADDRBIT8_HIGH_TOREAD,  ICM20649_ACCEL_CONFIG_1);

  Adafruit_BusIO_RegisterBits accel_dlpf_config =
    Adafruit_BusIO_RegisterBits(&accel_config_1, 3, 3);

  accel_dlpf_config.write("XXX");
  accel_dlpf_config.read();
*/



  bool Adafruit_ICM20649::setInterrupt1PinPolarity(bool active_high){
    Serial.print("Called INT1 Polarity ");
  }
  bool Adafruit_ICM20649::setI2CBypass(bool bypass_i2c){
    Serial.print("Bypassing I2C");
  }
