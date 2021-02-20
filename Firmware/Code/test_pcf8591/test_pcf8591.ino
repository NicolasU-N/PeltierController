//// DAC Demo Software for TI DAC081S101
//// EEforEveryone - 2020
//
////#include <SPI.h>
//#include <Wire.h>
//
////#define DAC_CS 9 //CS for DAC is on pin 9
////#define DELAYTIME 10 //delay, in seconds
//
//void setup() {
//  Serial.begin(9600);
//  Wire.begin();
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//
//  Serial.println("255");
//  DAC_SENDBYTE(255); //Output full magnitude (5V)
//  delay(500); //wait a second
//
//  Serial.println("128");
//  DAC_SENDBYTE(128); //Output half magnitude (2.5V)
//  delay(500); //wait a second
//
//  Serial.println("0");
//  DAC_SENDBYTE(0); //Output zero magnitude (0V)
//  delay(500); //wait a second
//
//  Serial.println("51");
//  DAC_SENDBYTE(51); //Output 20% magnitude (1V)
//  delay(500); //wait a second
//
//}
//
//
//void DAC_SENDBYTE(uint8_t data) {
//
//  uint8_t controlByte = 0b01000000; //Force into normal mode, then upper 4 bits of data
//  uint8_t upperdata = data<<0 | 0b00000000; //Force into normal mode, then upper 4 bits of data
//
//  Wire.beginTransmission(0b1001000); // Transmit to addres 0xD (00001101)  device address is specified in datasheet GND //(0x90 >> 1
//  Wire.write(controlByte);       //upper byte = mode select + upper 6 bits of data
//  Wire.write(upperdata);       //lower byte = lower 2 bits of data + zeros
//  Wire.endTransmission();      // stop transmitting
//
//}
//

/*
 * PCF8574A
 */

//// DAC Demo Software for TI DAC081S101
//// EEforEveryone - 2020
//
////#include <SPI.h>
//#include <Wire.h>
//
////#define DAC_CS 9 //CS for DAC is on pin 9
////#define DELAYTIME 10 //delay, in seconds
//
//void setup() {
//  Serial.begin(9600);
//  Wire.begin();
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//
//  Serial.println("255");
//  DAC_SENDBYTE(255); //Output full magnitude (5V)
//  delay(500); //wait a second
//
//  Serial.println("128");
//  DAC_SENDBYTE(128); //Output half magnitude (2.5V)
//  delay(500); //wait a second
//
//  Serial.println("0");
//  DAC_SENDBYTE(0); //Output zero magnitude (0V)
//  delay(500); //wait a second
//
//  Serial.println("51");
//  DAC_SENDBYTE(51); //Output 20% magnitude (1V)
//  delay(500); //wait a second
//
//}
//
//
//void DAC_SENDBYTE(uint8_t data) {
//
//  //uint8_t controlByte = 0b01000000; //Force into normal mode, then upper 4 bits of data
//  //uint8_t upperdata = data<<0 | 0b00000000; //Force into normal mode, then upper 4 bits of data
//
//  Wire.beginTransmission(0b0111000); // Transmit to addres 0xD (00001101)  device address is specified in datasheet GND //(0x90 >> 1
//  //Wire.write(controlByte);       //upper byte = mode select + upper 6 bits of data
//  Wire.write(0b11111111);       //lower byte = lower 2 bits of data + zeros
//  Wire.endTransmission();      // stop transmitting
//
//}


/*
 * I2C SCAN
 */


//#include <Wire.h>
//
//void setup()
//{
//  Wire.begin();
//
//  Serial.begin(9600);
//  while (!Serial);             // Leonardo: wait for serial monitor
//  Serial.println("\nI2C Scanner");
//}
//
//
//void loop()
//{
//  byte error, address;
//  int nDevices;
//
//  Serial.println("Scanning...");
//
//  nDevices = 0;
//  for(address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
//
//    if (error == 0)
//    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
//
//      nDevices++;
//    }
//    else if (error==4)
//    {
//      Serial.print("Unknown error at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.println(address,HEX);
//    }
//  }
//  if (nDevices == 0)
//    Serial.println("No I2C devices found\n");
//  else
//    Serial.println("done\n");
//
//  delay(5000);           // wait 5 seconds for next scan
//}
