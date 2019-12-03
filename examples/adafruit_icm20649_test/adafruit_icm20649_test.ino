// Basic demo for accelerometer readings from Adafruit MSA301

#include <Wire.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20649 icm; // TODO FIX NAME

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20649 test!");
  
  // Try to initialize!
  if (! icm.begin()) {
    Serial.println("Failed to find ICM20649 chip");
    while (1) { delay(10); }
  }
  Serial.println("ICM20649 Found!");
  //icm.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
    case ICM20649_ACCEL_RANGE_4_G: Serial.println("+-4G"); break;
    case ICM20649_ACCEL_RANGE_8_G: Serial.println("+-8G"); break;
    case ICM20649_ACCEL_RANGE_16_G: Serial.println("+-16G"); break;
    case ICM20649_ACCEL_RANGE_30_G: Serial.println("+-30G"); break;
  }

  //icm.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
    case ICM20649_GYRO_RANGE_500_DPS: Serial.println("500 degrees/s"); break;
    case ICM20649_GYRO_RANGE_1000_DPS: Serial.println("1000 degrees/s"); break;
    case ICM20649_GYRO_RANGE_2000_DPS: Serial.println("2000 degrees/s"); break;
    case ICM20649_GYRO_RANGE_4000_DPS: Serial.println("4000 degrees/s"); break;
  }

  icm.setGyroRateDivisor(0);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  uint16_t gyro_rate = 1100/(1.0+gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");Serial.println(gyro_rate);
}

void loop() {

//  /* Get a new normalized sensor event */ 
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp);
    
//    Serial.print("\t\tTemperature "); Serial.print(temp.temperature);
//    Serial.println(" deg C");
//
//    /* Display the results (acceleration is measured in m/s^2) */
//    Serial.print("\t\tAccel X: "); Serial.print(accel.acceleration.x);
//    Serial.print(" \tY: "); Serial.print(accel.acceleration.y);
//    Serial.print(" \tZ: "); Serial.print(accel.acceleration.z);
//    Serial.println(" m/s^2 ");
//    
//    /* Display the results (acceleration is measured in m/s^2) */
//    Serial.print("\t\tGyro X: "); Serial.print(gyro.gyro.x);
//    Serial.print(" \tY: "); Serial.print(gyro.gyro.y);
//    Serial.print(" \tZ: "); Serial.print(gyro.gyro.z);
//    Serial.println(" degrees/s ");
//    Serial.println();
//   
//    delay(100);

//  Serial.print(temp.temperature);
//  
//      Serial.print(","); 
  Serial.print(accel.acceleration.x);
  Serial.print(","); Serial.print(accel.acceleration.y);
  Serial.print(","); Serial.print(accel.acceleration.z);

  Serial.print(",");
  Serial.print(gyro.gyro.x);
  Serial.print(","); Serial.print(gyro.gyro.y);
  Serial.print(","); Serial.print(gyro.gyro.z);
  Serial.println();
  
  delayMicroseconds(200);
}