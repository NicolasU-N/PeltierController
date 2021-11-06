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
#include <ADS1115_lite.h>


#define ADDR_DAC  0x0D
#define ADDR_PCF  0x38

ADS1115_lite ads3(ADS1115_ADDRESS_ADDR_SCL); // 0x48 addr pin connected to GND
ADS1115_lite ads(ADS1115_ADDRESS_ADDR_GND);
ADS1115_lite ads1(ADS1115_ADDRESS_ADDR_VDD);
ADS1115_lite ads2(ADS1115_ADDRESS_ADDR_SDA);

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



  ads.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads.setSampleRate(ADS1115_REG_CONFIG_DR_8SPS); // Set to the fastest MODE 860Samples per sec

  ads1.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads1.setSampleRate(ADS1115_REG_CONFIG_DR_8SPS); // Set to the fastest MODE 860Samples per sec

  ads2.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads2.setSampleRate(ADS1115_REG_CONFIG_DR_8SPS); // Set to the fastest MODE 860Samples per sec


  ads3.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads3.setSampleRate(ADS1115_REG_CONFIG_DR_8SPS); // Set to the fastest MODE 860Samples per sec

  Serial.print("->WELCOME");
}

void loop() {

  dacSendByte(127);
  pcf8574.digitalWrite(5, 0);

  delay(500);
  dacSendByte(255);
  pcf8574.digitalWrite(5, 1);

  delay(500);
  dacSendByte(127);
  pcf8574.digitalWrite(5, 0);

  Serial.print("ADS_3-> ");
  Serial.print(abs(getAds3(0)));

  Serial.print(" ||| ");
  Serial.print(abs(getAds3(1)));

  Serial.print(" ||| ");
  Serial.print(abs(getAds3(2)));

  Serial.print(" ||| ");
  Serial.println(abs(getAds3(3)));


  Serial.print("ADS_1-> ");
  Serial.print(abs(getAds1(0)));

  Serial.print(" ||| ");
  Serial.print(abs(getAds1(1)));

  Serial.print(" ||| ");
  Serial.print(abs(getAds1(2)));

  Serial.print(" ||| ");
  Serial.println(abs(getAds1(3)));


  Serial.print("ADS_2-> ");
  Serial.print(abs(getAds2(0)));

  Serial.print(" ||| ");
  Serial.print(abs(getAds2(1)));

  Serial.print(" ||| ");
  Serial.print(abs(getAds2(2)));

  Serial.print(" ||| ");
  Serial.println(abs(getAds2(3)));

  /*
    uint8_t val = pcf8574.digitalRead(6);
    uint8_t val1 = pcf8574.digitalRead(7);

    Serial.print("val");
    Serial.println(val);
    Serial.print(" ---- ");
    Serial.print("val1");
    Serial.println(val1);
  */
}

/*
  0...255
*/
void dacSendByte(uint8_t data1) {
  uint8_t upperdata = (data1 >> 4) & 0b00001111; //Force into normal mode, then upper 4 bits of data
  uint8_t lowerdata = (data1 << 4) & 0b11110000; //retain lower 4 bits of data, then 4 don't care

  Wire.beginTransmission(ADDR_DAC); // Transmit to addres 0xD (0001101)  device address is specified in datasheet GND
  Wire.write(upperdata);
  Wire.write(lowerdata);
  Wire.endTransmission();      // stop transmitting // Comment to test continuous mode
}

int16_t getAds3(uint8_t port)
{
  switch (port)
  {
    case 0:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  ads3.triggerConversion();    // Triggered mannually
  return ads3.getConversion(); // returns int16_t value
}

int16_t getAds0(uint8_t port)
{
  Serial.println("EN LINEA 167");
  switch (port)
  {
    case 0:
      Serial.println("EN CASE 1 ");
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  Serial.println("EN LINEA 184");
  //ads.triggerConversion();    // Triggered mannually
  Serial.println("VALOR RETURN");
  Serial.println(ads.getConversion());
  Serial.println("LINEA 188");
  return ads.getConversion(); // returns int16_t value
}

int16_t getAds1(uint8_t port)
{
  switch (port)
  {
    case 0:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  ads1.triggerConversion();    // Triggered mannually
  return ads1.getConversion(); // returns int16_t value
}

int16_t getAds2(uint8_t port)
{
  switch (port)
  {
    case 0:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  ads2.triggerConversion(); // Triggered mannually
  return ads2.getConversion();
}
