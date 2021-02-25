/**
   PELTIER TEMPERATURE CONTROL
   Board: ESP32-WROOM32
   Last Modified: 25 Feb 2021

   Links:
   https://e2e.ti.com/support/data-converters/f/73/t/591378?DAC121C081-Issue-with-DAC-programming-in-Energia-and-Tiva-C-series
   https://e2e.ti.com/support/data-converters/f/73/t/581078
   https://www.overclock.net/threads/very-simple-koolance-arduino-flow-meter.1625912/
   https://forum.seeedstudio.com/t/arduino-controlled-pc-water-cooling-system-info-center/14484
   https://groups.google.com/g/diy-pid-control/c/oBwd3F-Hnac

   //opcional
   https://forum.arduino.cc/index.php?topic=604731.0


   TODO
   ====
 * * .
 * * Realizar funcion para escuchar del puerto serie.
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
#include <Adafruit_ADS1015.h>
#include "PCF8574.h"

#define ADDR_DAC 0b0001101
#define ADDR_PCF 0b0111000
#define ADDR_ADS0 0b1001000
#define ADDR_ADS1 0b1001001
#define ADDR_ADS2 0b1001010
#define ADDR_ADS3 0b1001011

TaskHandle_t Task1;

// Set i2c HEX address
PCF8574 pcf8574(ADDR_PCF);
Adafruit_ADS1115 ads0(ADDR_ADS0);
Adafruit_ADS1115 ads1(ADDR_ADS1);
Adafruit_ADS1115 ads2(ADDR_ADS2);
Adafruit_ADS1115 ads3(ADDR_ADS3);

volatile int countR = 0;
volatile int countF = 0;
uint16_t pumpRpm = 0;
uint16_t flowLPM = 0;

//
float tempsPel[10];

float r1 = 10000;
float logR2, r2, temp;
float c1 = 0.8483323763e-03, c2 = 2.581286591e-04, c3 = 1.641220112e-07;
//https://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm


////////////////////////
//       STATES       //
////////////////////////
#define INITMODE    0
#define STARTMODE   1
#define IDLEMODE    2
#define COOLINGMODE 3

uint8_t state = INITMODE;

void codeForTask1( void * parameter )
{
  for (;;) { // loop
    //  blink(LED1, 1000);
    delay(50);
    Serial.println("Task 1: ");
  }
}

void setup() {
  pcf8574.pinMode(P1, INPUT);
  pcf8574.pinMode(P2, INPUT);
  pcf8574.pinMode(P3, INPUT);
  pcf8574.pinMode(P5, OUTPUT);
  pcf8574.pinMode(P7, INPUT_PULLUP);
  pcf8574.pinMode(P6, INPUT);
  pcf8574.begin();
  ads0.begin();
  ads1.begin();
  ads2.begin();
  ads3.begin();

  xTaskCreatePinnedToCore(
    codeForTask1,
    "Send feedback of machine status by Serial port",
    1000,
    NULL,
    1,
    &Task1,
    1);

}

void loop() {

  switch (state) {
    case INITMODE:
      // statements
      break;
    case STARTMODE:
      // statements
      break;
    case IDLEMODE:
      // statements
      break;
    case COOLINGMODE:
      // statements
      break;
  }

  //CONTROL TEMP
  //Implement state switch


  readCountRpmFlow();
  readPumpRpmAndFlowMeter();

}

/*

*/
void dacSendByte(uint8_t data) {
  uint8_t upperdata = (data >> 4) & 0b00001111; //Force into normal mode, then upper 4 bits of data
  uint8_t lowerdata = (data << 4) & 0b11110000; //retain lower 4 bits of data, then 4 don't care

  Wire.beginTransmission(ADDR_DAC); // Transmit to addres 0xD (0001101)  device address is specified in datasheet GND
  Wire.write(upperdata);
  Wire.write(lowerdata);
  Wire.endTransmission();      // stop transmitting
}

void  readPumpRpmAndFlowMeter() {
  static unsigned long previousMillis = 0;
  if ((millis() - previousMillis) >= 1000) { // Millise
    previousMillis += millis();
    pumpRpm = countR * 30; // 2 pulses per turn (60/2)
    countR = 0;
    flowLPM = countF * 0.283; // LPM = 0.283 x Hz
    countF = 0;
  }
}

/*

*/
void readCountRpmFlow() {
  static unsigned long previousMillis = 0;
  if ((millis() - previousMillis) >= 1) { // Millise
    previousMillis += millis();
    uint8_t val = pcf8574.digitalRead(P6);
    if (val == HIGH) countR++;
    uint8_t val1 = pcf8574.digitalRead(P7);
    if (val1 == HIGH) countF++;
  }
}

/*

*/
uint8_t readSpareDigitalInput(uint8_t port) {
  uint8_t val = pcf8574.digitalRead(port);
  return val;
}

/*

*/
void writeOnPump(uint8_t state) {
  pcf8574.digitalWrite(P5, state);
}

/*
  port 0...3
*/
float readTempSenAp(uint8_t port, bool isNTC) {
  if (isNTC) {
    float vout = (float)ads3.readADC_SingleEnded(port) * (3.3 / 32767); //0...32767 resolution
    float R_NTC = (vout * 10000) / (3.3 - vout); //calculating the resistance of the thermistor
    float Temp_K = (298.15 * 3293.79) / (298.15 * log(R_NTC / 10000) + 3293.79); //Temperature in Kelvin B_param and T0 = 25C TO K
    float Temp_C = Temp_K - 273.15; //converting into Celsius
    return Temp_C;
  } else {
    return (float)ads3.readADC_SingleEnded(port);
  }
}

/*
  0...9
*/
float readTempPel(uint8_t number) {
  float vout = 0;
  if (number >= 0 and number <= 3) { //T1-T4
    vout = (float)ads0.readADC_SingleEnded(number) * (3.3 / 32767); //0...32767 resolution
  } else if (number > 3  and number <= 7) { //T5-T8
    vout = (float)ads1.readADC_SingleEnded(number - 4) * (3.3 / 32767); //0...32767 resolution
  } else if (number > 7) { //T9-T10
    vout = (float)ads2.readADC_SingleEnded(number - 8) * (3.3 / 32767); //0...32767 resolution
  }
  float R_NTC = (vout * 10000) / (3.3 - vout); //calculating the resistance of the thermistor
  float Temp_K = (298.15 * 3799.42) / (298.15 * log(R_NTC / 10000) + 3799.42); //Temperature in Kelvin B_param and T0 = 25C TO K
  float Temp_C = Temp_K - 273.15; //converting into Celsius
  return Temp_C;
}

/*
   0...3
*/
float writeTempPel(uint8_t number) {
  float vout = 0;
  if (number >= 0 and number <= 3) { //T1-T4
    vout = (float)ads0.readADC_SingleEnded(number) * (3.3 / 32767); //0...32767 resolution
  } else if (number > 3  and number <= 7) { //T5-T8
    vout = (float)ads1.readADC_SingleEnded(number - 4) * (3.3 / 32767); //0...32767 resolution
  } else if (number > 7) { //T9-T10
    vout = (float)ads2.readADC_SingleEnded(number - 8) * (3.3 / 32767); //0...32767 resolution
  }

  if (sensors.getTempCByIndex(0) < lowtemp) {
    digitalWrite(relay1, HIGH);
    // ADD PWM

  }
  if (sensors.getTempCByIndex(0) > hightemp) {
    digitalWrite(relay1, LOW);

  }
}

/*
  int PWM_FREQUENCY = 1000; // this variable is used to define the time period
  int PWM_CHANNEL = 0; // this variable is used to select the channel number
  int PWM_RESOUTION = 8; // this will define the resolution of the signal which is 8 in this case
  int GPIOPIN = 15 ; // GPIO to which we want to attach this channel signal
  int dutyCycle = 127; // it will define the width of signal or also the one time

  void setup()
  {

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOUTION);
  ledcAttachPin(GPIOPIN, PWM_CHANNEL);


  }

  void loop()
  {

  ledcWrite(PWM_CHANNEL, dutyCycle);

  }
*/


/*
  readAllSensor
  readSerialPort
*/
