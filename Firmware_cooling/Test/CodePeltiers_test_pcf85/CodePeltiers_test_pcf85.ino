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
#include "PCF8574.h"


#define ADDR_PCF  0x38

// Set i2c HEX address
PCF8574 pcf8574(&Wire, 0x38);

void setup() {
  Serial.begin(115200);

  pcf8574.pinMode(P1, INPUT);
  pcf8574.pinMode(P2, INPUT);
  pcf8574.pinMode(P3, INPUT);
  pcf8574.pinMode(P5, OUTPUT, LOW);
  pcf8574.pinMode(P7, INPUT);
  pcf8574.pinMode(P6, INPUT);

  Wire.begin();
  pcf8574.begin();



  Serial.print("->WELCOME");
}

void loop() {
  static unsigned long previousMillis = 0;
  
  if ((millis() - previousMillis) >= 1000) { // Millise
    pcf8574.digitalWrite(5, 0);
    delay(300);
    pcf8574.digitalWrite(5, 1);
    delay(300);
    pcf8574.digitalWrite(5, 0);
  }
  
  uint8_t val = pcf8574.digitalRead(6);
  uint8_t val1 = pcf8574.digitalRead(7);

  Serial.print("val");
  Serial.println(val);
  Serial.print(" ---- ");
  Serial.print("val1");
  Serial.println(val1);
}
