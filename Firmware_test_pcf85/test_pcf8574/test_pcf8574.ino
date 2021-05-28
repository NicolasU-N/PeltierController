#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Set i2c address
PCF8574 pcf8574(0x20); //&Wire,

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  //pcf8574.pinMode(P7, OUTPUT);
  pcf8574.pinMode(P4, OUTPUT);
  Wire.begin();
  if (pcf8574.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  pcf8574.digitalWrite(P4, 1);
  delay(400);
  pcf8574.digitalWrite(P4, 0);
  delay(400);
}
