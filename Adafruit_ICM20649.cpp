
/*!
 *  @file Adafruit_ICM20649.cpp
 *
 *  @mainpage Adafruit ICM20649 6-DoF Wide-Range Accelerometer and Gyro library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit ICM20649 6-DoF Wide-Range Accelerometer and Gyro library
 * 
 * 	This is a library for the Adafruit ICM20649 breakout:
 * 	https://www.adafruit.com/product/XXX
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
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_ICM20649::begin(uint8_t i2c_address, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init();
}

boolean Adafruit_ICM20649::_init(void) {
  Adafruit_BusIO_Register chip_id = 
    Adafruit_BusIO_Register(i2c_dev, ICM20649_WHOAMI, 2);

  // make sure we're talking to the right chip
  if (chip_id.read() != ICM20649_CHIP_ID) {
    return false;
  }
  // do any software reset or other initial setup
  return true;
}

/*********** typdef enum getter with bitfield *********************/

// /**************************************************************************/
// /*!
//     @brief Gets EXAMPLE VALUE.
//     @returns The EXAMPLE VALUE.
// */
// ICM20649_example_t Adafruit_ICM20649::getEXAMPLE(void){
    
//     Adafruit_BusIO_Register example_register =
//       // Adafruit_I2CDevice pointer, address, number of bytes
//       Adafruit_BusIO_Register(i2c_dev, ICM20649_EXAMPLE_REG, 2); 

//     Adafruit_BusIO_RegisterBits example_bitfield =
//       // register pointer, number of bits, shift
//       Adafruit_BusIO_RegisterBits(&example_register, 3, 1);
//     write(example_value);
//     return (ICM20649_example_t)example_bitfield.read();
// }

// /*********** typdef enum setter with bitfield  *********************/

// /**************************************************************************/
// /*!
//     @brief Sets EXAMPLE VALUE.
//     @param  example_value
//             The EXAMPLE used to EXAMPLE. Must be a
//             `ICM20649_example_t`.
// */
// void Adafruit_ICM20649::setEXAMPLE(ICM20649_example_t example_value){
    
//     Adafruit_BusIO_Register example_register =
//       // Adafruit_I2CDevice pointer, address, number of bytes
//       Adafruit_BusIO_Register(i2c_dev, ICM20649_EXAMPLE_REG, 2); 

//     Adafruit_BusIO_RegisterBits example_bitfield =
//       // register pointer, number of bits, shift
//       Adafruit_BusIO_RegisterBits(&example_register, 3, 1);
//     example_bitfield.write(example_value);
// }