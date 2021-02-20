/**
   PELTIER TEMPERATURE CONTROL
   Board: ESP32-WROOM32
   Last Modified: 16 Feb 2021

   Links:
   https://e2e.ti.com/support/data-converters/f/73/t/591378?DAC121C081-Issue-with-DAC-programming-in-Energia-and-Tiva-C-series
   https://e2e.ti.com/support/data-converters/f/73/t/581078
   https://www.overclock.net/threads/very-simple-koolance-arduino-flow-meter.1625912/
   https://koolance.com/coolant-flow-meter-stainless-steel-with-temperature-sensor-sen-fm18t10
   https://forum.seeedstudio.com/t/arduino-controlled-pc-water-cooling-system-info-center/14484
   https://groups.google.com/g/diy-pid-control/c/oBwd3F-Hnac

   https://github.com/adafruit/Adafruit_ADS1X15/blob/master/examples/singleended/singleended.ino

   https://benkrasnow.blogspot.com/2009/09/peltier-power-supply-and-integrated-pid.html //opcional
   https://forum.arduino.cc/index.php?topic=604731.0

   //busqueda CONTROLLER PELTIERS ARDUINO
   
   TODO
   ====
 * * Realizar funciones para las lecturas de sensores.
 * * Realizar funcion para escuchar del puerto serie.
 * * Crear TaskHandler.
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
#include <Adafruit_ADS1015.h>
#include "PCF8574.h"

#define ADDR_DAC 0b0001101
#define ADDR_PCF 0b0111000
#define ADDR_ADS0 0b1001000
#define ADDR_ADS1 0b1001001
#define ADDR_ADS2 0b1001010
#define ADDR_ADS3 0b1001011

// Set i2c HEX address
PCF8574 pcf8574a(ADDR_PCF);
unsigned long timeElapsed;

//void setup(){
//  Serial.begin(9600);
//
//  pcf8574.pinMode(P0, INPUT);
//  pcf8574.pinMode(P1, OUTPUT);
//
//  pcf8574.begin();
//}
//
//void loop(){
//  uint8_t val = pcf8574.digitalRead(P1);            // Read the value of pin P0
//  if (val == HIGH)  pcf8574.digitalWrite(P0, HIGH); // If Button is Pressed
//  else              pcf8574.digitalWrite(P0, LOW);  // When Button is Released
//  delay(50);
//}


void setup() {
  // put your setup code here, to run once:
  pcf8574.begin();
}

void loop() {

  
  // put your main code here, to run repeatedly:

}


void DAC_SENDBYTE(uint8_t data) {
  uint8_t upperdata = (data >> 4) & 0b00001111; //Force into normal mode, then upper 4 bits of data
  uint8_t lowerdata = (data << 4) & 0b11110000; //retain lower 4 bits of data, then 4 don't care

  Wire.beginTransmission(ADDR_DAC); // Transmit to addres 0xD (0001101)  device address is specified in datasheet GND
  Wire.write(upperdata);      
  Wire.write(lowerdata);       
  Wire.endTransmission();      // stop transmitting
}
