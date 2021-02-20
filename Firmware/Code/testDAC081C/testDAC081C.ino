// DAC Demo Software for TI DAC081C081

#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("255");
  DAC_SENDBYTE(255); //Output full magnitude (5V)
  delay(500); //wait a second

  Serial.println("128");
  DAC_SENDBYTE(128); //Output half magnitude (2.5V)
  delay(500); //wait a second

  Serial.println("0");
  DAC_SENDBYTE(0); //Output zero magnitude (0V)
  delay(500); //wait a second

  Serial.println("51");
  DAC_SENDBYTE(51); //Output 20% magnitude (1V)
  delay(500); //wait a second

}


void DAC_SENDBYTE(uint8_t data) {
  uint8_t upperdata = (data >> 4) & 0b00001111; //Force into normal mode, then upper 4 bits of data
  uint8_t lowerdata = (data << 4) & 0b11110000; //retain lower 4 bits of data, then 4 don't care

  Wire.beginTransmission(0xD); // Transmit to addres 0xD (0001101)  device address is specified in datasheet GND
  Wire.write(upperdata);       //upper byte = mode select + upper 6 bits of data
  Wire.write(lowerdata);       //lower byte = lower 2 bits of data + zeros
  Wire.endTransmission();      // stop transmitting
}
