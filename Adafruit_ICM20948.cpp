
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
  Serial.print("done with init: ");Serial.println(init_success);
  // if (! _setupMag()){
  //   Serial.println("failed to setup mag");
  //   return false;
  // } 

  uint8_t buffer[2];

  _setBank(0);

  buffer[0] = ICM20X_REG_INT_PIN_CFG;
  buffer[1] = 0x00;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("set PIN CONFIG");
  _setBank(3);

  Serial.println("set i2c clk & stop");
  // WRITE	ICM20948_I2C_MST_CTRL	0x17		I2C_MST_P_ NSR	I2C_MST_CLK[3:0]	7
  buffer[0] = ICM20948_I2C_MST_CTRL;
  buffer[1] = 0x17;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("I2C_MST_EN=1");

   _setBank(0);
  // READ	USER_CTRL	0x00
  // WRITE	USER_CTRL	0x20
  buffer[0] = ICM20X_USER_CTRL;
  buffer[1] = 0x20;  // set I2C_MST_EN
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

uint8_t idl, idh;
Serial.println("REad idH: 0x");
idl = _read_ext_reg(0x8C, 0x00);
Serial.println(idl, HEX);

Serial.println("REad idL: 0x");
idh = _read_ext_reg(0x8C, 0x01);
Serial.println(idh, HEX);

_setBank(3);

  Serial.println("slv4 addr set ");
  // WRITE	I2C_SLV4_ADDR	0x0C
  buffer[0] = ICM20948_I2C_SLV4_ADDR;
  buffer[1] = 0x0C;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  Serial.println("slv4 reg set ");
  // WRITE	ICM20948_I2C_SLV4_REG	0x31	CNTL2
  buffer[0] = ICM20948_I2C_SLV4_REG;
  buffer[1] = 0x31;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 DO set ");
  // WRITE	ICM20948_I2C_SLV4_DO	0x08
  buffer[0] = ICM20948_I2C_SLV4_DO;
  buffer[1] = 0x08;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 CTRL /active set");
  // WRITE	ICM20948_I2C_SLV4_CTRL	0x80
  buffer[0] = ICM20948_I2C_SLV4_CTRL;
  buffer[1] = 0x80;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

Serial.println("checking slave 4 status");
  _setBank(0);
  uint8_t addrbuffer[2] = {(uint8_t)ICM20948_I2C_MST_STATUS, (uint8_t)0};
  buffer[0] = 0;
  buffer[1] = 0;
  while (buffer[0] != 0x40){
    Serial.print("not ready: buffer[0] = 0x"); Serial.println(buffer[0], HEX);
    Serial.print("not ready: buffer[1] = 0x"); Serial.println(buffer[1], HEX);
    i2c_dev->write_then_read(addrbuffer, 1, buffer, 1);
    delay(100);
  }
  // SET MAG ADDRESS to read from map
  _setBank(3);
  // WRITE	ICM20948_I2C_SLV0_ADDR	0x8C
  Serial.println("slv4 addr set ");

  buffer[0] = ICM20948_I2C_SLV0_ADDR;
  buffer[1] = 0x8C;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 reg set ");
  // WRITE	ICM20948_I2C_SLV0_REG	0x10
  buffer[0] = ICM20948_I2C_SLV0_REG;
  buffer[1] = 0x10;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 CTRL /active set");

  // WRITE	ICM20948_I2C_SLV0_CTRL	0x89
  buffer[0] = ICM20948_I2C_SLV0_CTRL;
  buffer[1] = 0x89;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  return true;





  return init_success;
}

bool Adafruit_ICM20948::_setupMag(void){
  


  uint8_t buffer[2];

  // _setBank(0);

  // buffer[0] = ICM20X_REG_INT_PIN_CFG;
  // buffer[1] = 0x00;
  // if (!i2c_dev->write(buffer, 2)) {
  //   return false;
  // }
  // Serial.println("set PIN CONFIG");
  // _setBank(3);


  // // WRITE	ICM20948_I2C_MST_CTRL	0x17		I2C_MST_P_ NSR	I2C_MST_CLK[3:0]	7
  // buffer[0] = ICM20948_I2C_MST_CTRL;
  // buffer[1] = 0x17;
  // if (!i2c_dev->write(buffer, 2)) {
  //   return false;
  // }
  // Serial.println("set MST+CRTL");

  // WRITE	BANK_SET	0x00
    // _setBank(0);
  // READ	USER_CTRL	0x00
  // // WRITE	USER_CTRL	0x20
  // buffer[0] = ICM20X_USER_CTRL;
  // buffer[1] = 0x20;  // set I2C_MST_EN
  // if (!i2c_dev->write(buffer, 2)) {
  //   return false;
  // }

  // Serial.println("sI2C_MST_ENABLE");


// uint8_t idl, idh;
// idl = _read_ext_reg(0x8C, 0x00);
// Serial.print("REad idH: 0x");Serial.println(idl, HEX);
// idh = _read_ext_reg(0x8C, 0x01);
// Serial.print("REad idL: 0x");Serial.println(idh, HEX);





  /////////// SETUP DATA RATE FOR SLAVE4
  _setBank(3);

  // WRITE	I2C_SLV4_ADDR	0x0C
  buffer[0] = ICM20948_I2C_SLV4_ADDR;
  buffer[1] = 0x0C;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 addr set ");

  // WRITE	ICM20948_I2C_SLV4_REG	0x31	CNTL2
  buffer[0] = ICM20948_I2C_SLV4_REG;
  buffer[1] = 0x31;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 reg set ");
  // WRITE	ICM20948_I2C_SLV4_DO	0x08
  buffer[0] = ICM20948_I2C_SLV4_DO;
  buffer[1] = 0x08;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 DO set ");
  // WRITE	ICM20948_I2C_SLV4_CTRL	0x80
  buffer[0] = ICM20948_I2C_SLV4_CTRL;
  buffer[1] = 0x80;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("slv4 CTRL /active set");

  _setBank(0);
  uint8_t addrbuffer[2] = {(uint8_t)ICM20948_I2C_MST_STATUS, (uint8_t)0};
  buffer[0] = 0;
  buffer[1] = 0;
  while (buffer[0] != 0x40){
    Serial.print("not ready: buffer[0] = 0x"); Serial.println(buffer[0], HEX);
    Serial.print("not ready: buffer[1] = 0x"); Serial.println(buffer[1], HEX);
    i2c_dev->write_then_read(addrbuffer, 1, buffer, 1);
    delay(100);
  }
Serial.println("done checking slave 4 status");
  // SET MAG ADDRESS to read from map
  _setBank(3);
  // WRITE	ICM20948_I2C_SLV0_ADDR	0x8C

  buffer[0] = ICM20948_I2C_SLV0_ADDR;
  buffer[1] = 0x8C;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  // WRITE	ICM20948_I2C_SLV0_REG	0x10
  buffer[0] = ICM20948_I2C_SLV0_REG;
  buffer[1] = 0x10;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  // WRITE	ICM20948_I2C_SLV0_CTRL	0x89
  buffer[0] = ICM20948_I2C_SLV0_CTRL;
  buffer[1] = 0x89;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  return true;

}
uint8_t Adafruit_ICM20948::_read_ext_reg(uint8_t slv_addr, uint8_t reg_addr) {
  		// READ REAL ID BYTE		
      // WRITE		BANK_SET	0x30	B3
      // WRITE	13	I2C_SLV4_ADDR	0x8C	B3
      // WRITE	14	I2C_SLV4_REG	0x01	B3
      // WRITE	15	I2C_SLV4_CTRL	0x80	B3
    // WRITE		BANK_SET	0x00	B0
      // READ	17	I2C_MST_STATUS	0x00	B0

      // READ	17	I2C_MST_STATUS	0x00	B0

      // READ	17	I2C_MST_STATUS	0x40	B0
     // WRITE		BANK_SET	0x30	B3
      // READ	17	I2C_SLV4_DI	0x09	B3
  _setBank(3);
  uint8_t buffer[2];

  // WRITE	I2C_SLV4_ADDR	0x0C
  buffer[0] = ICM20948_I2C_SLV4_ADDR;
  buffer[1] = slv_addr;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("READEXT: slave address set ");


  buffer[0] = ICM20948_I2C_SLV4_REG;
  buffer[1] = reg_addr;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("READEXT: slave address set ");
  
  buffer[0] = ICM20948_I2C_SLV4_CTRL;
  buffer[1] = 0x80;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }
  Serial.println("READEXT: slave enabled! ");

  _setBank(0);


  Serial.println("READEXT: checking for done ");
  uint8_t addrbuffer[2] = {(uint8_t)ICM20948_I2C_MST_STATUS, (uint8_t)0};
  buffer[0] = 0;
  buffer[1] = 0;
  while (buffer[0] != 0x40){
    Serial.print("not ready: buffer[0] = 0x"); Serial.print(buffer[0], HEX);Serial.print(" buffer[1] = 0x"); Serial.println(buffer[1], HEX);
    i2c_dev->write_then_read(addrbuffer, 1, buffer, 1);
    delay(100);
  }

 _setBank(3);
  addrbuffer[0] = (uint8_t)ICM20948_I2C_SLV4_DI;
  // WRITE	ICM20948_I2C_SLV4_CTRL	0x80
  buffer[0] = 0;
  buffer[1] = 0;
  i2c_dev->write_then_read(addrbuffer, 1, buffer, 1);
  return buffer[0];
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
