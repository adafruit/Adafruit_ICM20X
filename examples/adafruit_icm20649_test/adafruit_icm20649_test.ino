// Basic demo for accelerometer readings from Adafruit MSA301

#include <Wire.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_BusIO.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20649 msa; // TODO FIX NAME

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20649 test!");
  
  // Try to initialize!
  if (! msa.begin()) {
    Serial.println("Failed to find ICM20649 chip");
    while (1) { delay(10); }
  }
  Serial.println("ICM20649 Found!");

  //msa.setDataRate(MSA301_DATARATE_31_25_HZ);
  //Serial.print("Data rate set to: ");
  //switch (msa.getDataRate()) {
  //  case MSA301_DATARATE_1_HZ: Serial.println("1 Hz"); break;
  //  case MSA301_DATARATE_1_95_HZ: Serial.println("1.95 Hz"); break;
  //  case MSA301_DATARATE_3_9_HZ: Serial.println("3.9 Hz"); break;
  //  case MSA301_DATARATE_7_81_HZ: Serial.println("7.81 Hz"); break;
  //  case MSA301_DATARATE_15_63_HZ: Serial.println("15.63 Hz"); break;
  //  case MSA301_DATARATE_31_25_HZ: Serial.println("31.25 Hz"); break;
  //  case MSA301_DATARATE_62_5_HZ: Serial.println("62.5 Hz"); break;
  //  case MSA301_DATARATE_125_HZ: Serial.println("125 Hz"); break;
  //  case MSA301_DATARATE_250_HZ: Serial.println("250 Hz"); break;
  //  case MSA301_DATARATE_500_HZ: Serial.println("500 Hz"); break;
  //  case MSA301_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  //}
}

void loop() {
  msa.read();      // get X Y and Z data at once
 
  delay(100); 
}
