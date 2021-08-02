/**
   PELTIER TEMPERATURE CONTROL
   Board: ESP32-WROOM32

   TODO
   ====
 * * .
 * * .
 * * .
*/

/**
   Pin Mapping:
   Physical Pin    Port Pin     Function
   ======================================
   41              IO1            TX
   40              IO3            RX
   42              IO21           I2C SDA
   39              IO22           I2C SCL
   25              IO16           SRC1
   36              IO23           S1_C1
   14              IO25           S1_C2
   27              IO17           SRC2
   30              IO11           S2_C1
   18              IO12           S2_C2
   35              IO18           SRC3
   11              IO32           S3_C1
   13              IO33           S3_C2
   38              IO19           SRC4
   20              IO13           S4_C1
   24              IO04           S4_C2

   Addresses:
   Device          Hex          Binary
   ======================================
   DAC081C081      0xD          0b0001101
   PCF8574ADWR     0x38         0b0111000
   ADS1115         0x48         0b1001000 (T1-T4 GND)
   ADS1115         0x49         0b1001001 (T5-T8 VDD)
   ADS1115         0x4A         0b1001010 (T9-T10 SDA)OUTPUT
   ADS1115         0x4B         0b1001011 (SENSORS SCL)
*/
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_ADS1015.h"

#define ADDR_ADS0 0x48
#define ADDR_ADS1 0x49
#define ADDR_ADS2 0x4A
#define ADDR_ADS3 0x4B

float tempsPel[10];

// Set i2c HEX address
Adafruit_ADS1115 ads0;
Adafruit_ADS1115 ads1(ADDR_ADS1);
Adafruit_ADS1115 ads2(ADDR_ADS2);
Adafruit_ADS1115 ads3(ADDR_ADS3);


void setup() {
  Serial.begin(115200);

   //Wire.setClock(400000);

  ads0.begin();
  ads1.begin();
  ads2.begin();
  ads3.begin();
}

void loop() {
  readAllSensors();
  delay(100);
}


void readAllSensors() {
  for (uint8_t i = 0; i < 3; i++) {
    tempsPel[i] = readTempPel(i);   
    delay(100);
  }
  delay(500);
}

/*
  0...9
*/
float readTempPel(uint8_t number) {
  float vout = 0;
  
  if (number >= 0 and number <= 3) { //T1-T4
    vout = (float)ads0.readADC_SingleEnded(number ); // * (3.3 / 32767); //0...32767 resolutio
    Serial.print("ADS1__");
    Serial.print(number);
    Serial.print("__");
    Serial.print(vout);
    Serial.print("__");
  }
  /*else if (number > 3  and number <= 7) { //T5-T8
    vout = (float)ads1.readADC_SingleEnded(number - 4);// * (3.3 / 32767); //0...32767 resolution
    Serial.print("ADS2__");
    Serial.print(number);
    Serial.print("__");
    Serial.print(vout);
    Serial.print("__");
  } else if (number > 7) { //T9-T10
    vout = (float)ads2.readADC_SingleEnded(number - 8);// * (3.3 / 32767); //0...32767 resolution
    Serial.print("ADS3__");
    Serial.print(number);
    Serial.print("__");
    Serial.print(vout);
    Serial.print("__");
  }
  
  //float Temp_C = Temp_K - 273.15; //converting into Celsius
  return vout;//Temp_C;
  */

  Serial.println("");
}
