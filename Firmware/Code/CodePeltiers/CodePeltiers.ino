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
   DAC081C081      0xD          00001101
   PCF8574ADWR     0x20
*/
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "PCF8574.h"

// Set i2c HEX address
PCF8574 pcf8574(0x20);
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


// data 0...255
void DAC8Bit(byte data) { 
  Wire.beginTransmission(0xD); // Transmit to addres 0xD (00001101)  device address is specified in datasheet GND
  Wire.write( (data >> 4) & 15);
  Wire.write( (data << 4) & 240);
  Wire.endTransmission();
}
