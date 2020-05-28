
#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_ICM20948.h"
#include "Adafruit_ICM20X.h"

/*!
 *    @brief  Instantiates a new ICM20948 class!
 */

Adafruit_ICM20948::Adafruit_ICM20948(void) {}
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
bool Adafruit_ICM20948::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                                  int32_t sensor_id) {

  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    Serial.println("I2C begin Failed");
    return false;
  }
  bool init_success = _init(sensor_id);
  if (!_setupMag()) {
    Serial.println("failed to setup mag");
    return false;
  }

  return init_success;
}

uint8_t Adafruit_ICM20948::getMagId(void) {
  // verify the magnetometer id
  return _read_ext_reg(0x8C, 0x01);
}
bool Adafruit_ICM20948::_setupMag(void) {
  uint8_t buffer[2];
  // self._magnetometer_enable()
  // self._bank = 0
  // self._bypass_i2c_master = False

  setI2CBypass(false);
  // self._bank = 3
  // self._i2c_master_control = 0x17

  // set I2C master frequency and turn off repeated starts
  _setBank(3);
  buffer[0] = ICM20948_I2C_MST_CTRL;
  // no repeated start, i2c master clock = 345.60kHz
  buffer[1] = 0x17;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  // self._bank = 0
  // self._i2c_master_enable = True
  // Enable I2C Master!
  buffer[0] = ICM20X_USER_CTRL;
  buffer[1] = 0x20;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  // self.magnetometer_data_rate = (
  //     MagDataRate.RATE_100HZ  # pylint: disable=no-member
  // )

  // check mag connection by reading the chip ID
  bool mag_setup_failed = true;
  for (int i = 0; i < I2C_MASTER_RESETS_BEFORE_FAIL; i++) {
    if (getMagId() != ICM20948_MAG_ID) {
      Serial.println(
          "\tFailed to read from magnetometer, resetting I2C master");
      _resetI2CMaster();
    } else {
      Serial.println("\tSuccessful setup of magnetometer");
      mag_setup_failed = false;
      break;
    }
  }
  if (mag_setup_failed) {
    return false;
  }

  // Error setting magnetometer data rate on external bus
  // failed to setup mag
  // Failed to find ICM20948 chip

  // set mag data rate
  bool mag_dr_status = _write_mag_reg(0x31, 0x08);
  if (!mag_dr_status) {
    Serial.println("Error setting magnetometer data rate on external bus");
    return false;
  }
  // if not self._mag_configured:
  //     return False

  // self._setup_mag_readout()

  // check to see if it's finished
  while (_read_mag_reg(0x32)) {
    delay(10);
  }
  /////// Set up Slave0 to proxy Mag readings
  _setBank(3);
  // set up slave0 to proxy reads to mag
  buffer[0] = ICM20948_I2C_SLV0_ADDR;
  buffer[1] = 0x8C;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  buffer[0] = ICM20948_I2C_SLV0_REG;
  buffer[1] = 0x10;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  buffer[0] = ICM20948_I2C_SLV0_CTRL;
  buffer[1] = 0x89; // enable, read 9 bytes
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  return true;
}

//  // Send a software reset to the magnetometer
//           if (!_write_mag_reg(0x32, 0x01)) {
//             Serial.print("Error writing to external register");
//             return false;
//           }

void Adafruit_ICM20948::_resetI2CMaster(void) {

  _setBank(0);
  Adafruit_BusIO_Register user_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20X_USER_CTRL);

  Adafruit_BusIO_RegisterBits i2c_master_reset_bit =
      Adafruit_BusIO_RegisterBits(&user_ctrl, 1, 1);

  i2c_master_reset_bit.write(true);
  while (i2c_master_reset_bit.read()) {
    delay(10);
  }
  delay(100);
}
/**
 * @brief
 *
 * @param slv_addr
 * @param reg_addr
 * @param num_finished_checks
 * @return uint8_t
 */
uint8_t Adafruit_ICM20948::_read_mag_reg(uint8_t reg_addr) {
  return _read_ext_reg(0x8C, reg_addr);
}

/**
 * @brief
 *
 * @param slv_addr
 * @param reg_addr
 * @param value
 * @param num_finished_checks
 * @return true
 * @return false
 */
bool Adafruit_ICM20948::_write_mag_reg(uint8_t reg_addr, uint8_t value) {
  return _write_ext_reg(0x0C, reg_addr, value);
}

uint8_t Adafruit_ICM20948::_read_ext_reg(uint8_t slv_addr, uint8_t reg_addr) {

  slv_addr |= 0x80; // set high bit for read

  uint8_t buffer[2];
  uint8_t tries = 0;
  _setBank(3);

  // set the I2C address on the external bus
  buffer[0] = ICM20948_I2C_SLV4_ADDR;
  buffer[1] = slv_addr;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  // specify which register we're reading
  buffer[0] = ICM20948_I2C_SLV4_REG;
  buffer[1] = reg_addr;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  // enable the read!
  buffer[0] = ICM20948_I2C_SLV4_CTRL;
  buffer[1] = 0x80;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  _setBank(0);
  uint8_t addrbuffer[2] = {(uint8_t)ICM20948_I2C_MST_STATUS, (uint8_t)0};
  buffer[0] = 0;
  buffer[1] = 0;
  // wait until the operation is finished
  while (buffer[0] != 0x40) { // this bit is slave4 finished!
    i2c_dev->write_then_read(addrbuffer, 1, buffer, 1);
    tries++;
    if (tries >= NUM_FINISHED_CHECKS) {
      _resetI2CMaster();
      return false;
    }
  }
  // read the reading
  _setBank(3);
  addrbuffer[0] = (uint8_t)ICM20948_I2C_SLV4_DI;
  buffer[0] = 0;
  buffer[1] = 0;
  i2c_dev->write_then_read(addrbuffer, 1, buffer, 1);
  return buffer[0];
}

bool Adafruit_ICM20948::_write_ext_reg(uint8_t slv_addr, uint8_t reg_addr,
                                       uint8_t value) {
  uint8_t buffer[2];

  // Adafruit_BusIO_Register slv4_addr = Adafruit_BusIO_Register(
  //   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20948_I2C_SLV4_ADDR);

  // Adafruit_BusIO_Register slv4_addr = Adafruit_BusIO_Register(
  //   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20948_I2C_SLV4_REG);

  // Adafruit_BusIO_Register slv4_addr = Adafruit_BusIO_Register(
  //   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20948_I2C_SLV4_DO);

  // Adafruit_BusIO_Register slv4_addr = Adafruit_BusIO_Register(
  //   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20948_I2C_SLV4_CTRL);

  _setBank(3);
  uint8_t tries = 0;
  // set the I2C address on the external bus
  buffer[0] = ICM20948_I2C_SLV4_ADDR;
  buffer[1] = slv_addr;
  if (!i2c_dev->write(buffer, 2)) {
    Serial.println("bad addr set");
    return false;
  }
  // specify which register we're writing to
  buffer[0] = ICM20948_I2C_SLV4_REG;
  buffer[1] = reg_addr;
  if (!i2c_dev->write(buffer, 2)) {
    Serial.println("bad reg set");
    return false;
  }
  // set the data to write
  buffer[0] = ICM20948_I2C_SLV4_DO;
  buffer[1] = value;
  if (!i2c_dev->write(buffer, 2)) {
    Serial.println("bad DO set");
    return false;
  }
  // start writing
  buffer[0] = ICM20948_I2C_SLV4_CTRL;
  buffer[1] = 0x80;
  if (!i2c_dev->write(buffer, 2)) {
    Serial.println("bad CTRL set");
    return false;
  }

  _setBank(0);
  uint8_t addrbuffer[2] = {(uint8_t)ICM20948_I2C_MST_STATUS, (uint8_t)0};
  buffer[0] = 0;
  buffer[1] = 0;
  // wait until the operation is finished
  while (buffer[0] != 0x40) {
    i2c_dev->write_then_read(addrbuffer, 1, buffer, 1);
    tries++;
    if (tries >= NUM_FINISHED_CHECKS) {
      Serial.println("failed to get i2c master finished signal");
      return false;
    }
  }
  return true;
}
void Adafruit_ICM20948::_scale_values(void) {

  icm20948_gyro_range_t gyro_range = getGyroRange();
  icm20948_accel_range_t accel_range = getAccelRange();
  float accel_scale = 1.0;
  float gyro_scale = 1.0;

  if (gyro_range == ICM20948_GYRO_RANGE_250_DPS)
    gyro_scale = 131.0;
  if (gyro_range == ICM20948_GYRO_RANGE_500_DPS)
    gyro_scale = 65.5;
  if (gyro_range == ICM20948_GYRO_RANGE_1000_DPS)
    gyro_scale = 32.8;
  if (gyro_range == ICM20948_GYRO_RANGE_2000_DPS)
    gyro_scale = 16.4;

  if (accel_range == ICM20948_ACCEL_RANGE_2_G)
    accel_scale = 16384.0;
  if (accel_range == ICM20948_ACCEL_RANGE_4_G)
    accel_scale = 8192.0;
  if (accel_range == ICM20948_ACCEL_RANGE_8_G)
    accel_scale = 4096.0;
  if (accel_range == ICM20948_ACCEL_RANGE_16_G)
    accel_scale = 2048.0;

  gyroX = rawGyroX / gyro_scale;
  gyroY = rawGyroY / gyro_scale;
  gyroZ = rawGyroZ / gyro_scale;

  accX = rawAccX / accel_scale;
  accY = rawAccY / accel_scale;
  accZ = rawAccZ / accel_scale;

  magX = rawMagX * ICM20948_UT_PER_LSB;
  magY = rawMagY * ICM20948_UT_PER_LSB;
  magZ = rawMagZ * ICM20948_UT_PER_LSB;
}

/**************************************************************************/
/*!
    @brief Get the accelerometer's measurement range.
    @returns The accelerometer's measurement range (`icm20948_accel_range_t`).
*/
icm20948_accel_range_t Adafruit_ICM20948::getAccelRange(void) {
  return (icm20948_accel_range_t)readAccelRange();
}

/**************************************************************************/
/*!

    @brief Sets the accelerometer's measurement range.
    @param  new_accel_range
            Measurement range to be set. Must be an
            `icm20948_accel_range_t`.
*/
void Adafruit_ICM20948::setAccelRange(icm20948_accel_range_t new_accel_range) {
  writeAccelRange((uint8_t)new_accel_range);
}

/**************************************************************************/
/*!
    @brief Get the gyro's measurement range.
    @returns The gyro's measurement range (`icm20948_gyro_range_t`).
*/
icm20948_gyro_range_t Adafruit_ICM20948::getGyroRange(void) {
  return (icm20948_gyro_range_t)readGyroRange();
}

/**************************************************************************/
/*!

    @brief Sets the gyro's measurement range.
    @param  new_gyro_range
            Measurement range to be set. Must be an
            `icm20948_gyro_range_t`.
*/
void Adafruit_ICM20948::setGyroRange(icm20948_gyro_range_t new_gyro_range) {
  writeGyroRange((uint8_t)new_gyro_range);
}
