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
#include <Adafruit_ADS1015.h>
#include "PCF8574.h"
#include <Separador.h>

#define SRC1  16
#define S1_C1 23
#define S1_C2 25
#define SRC2  17
#define S2_C1 11
#define S2_C2 12
#define SRC3  18
#define S3_C1 32
#define S3_C2 33
#define SRC4  19
#define S4_C1 13
#define S4_C2 4

#define ADDR_DAC  0b0001101
#define ADDR_PCF  0b0111000
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

Separador s;

volatile int countR = 0;
volatile int countF = 0;

float pumpState = 0;
float pumpRpm = 0; //uint_fast16_t
float flowLPM = 0;
// Temps peltiers 0...7 Heat 8..9 Cool
float tempsPel[10];
float spareDigitalInput[3]; //uint_fast8_t
float tempSenAp[4];
uint8_t dataDac = 0;

float r1 = 10000;
float logR2, r2, temp;
float c1 = 0.8483323763e-03, c2 = 2.581286591e-04, c3 = 1.641220112e-07;
//https://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm

uint_fast16_t PWM_FREQUENCY = 1000; // this variable is used to define the time period
uint_fast8_t PWM_RESOUTION = 8; // this will define the resolution of the signal which is 8 in this case
uint_fast8_t PWM_CHANNEL[] = {0, 1, 2, 3}; // this variable is used to select the channel number
uint_fast8_t GPIOPIN[] = {SRC1, SRC2, SRC3, SRC4}; // GPIO to which we want to attach this channel signal
//uint_fast16_t dutyCycle = 0; // it will define the width of signal or also the one time

String _bytes;

float idleTemp = 10; //default
float wcTemp = 6; //default

////////////////////////
//       STATES       //
////////////////////////
#define INITMODE    0
#define IDLEMODE    1
#define COOLINGMODE 2
#define STOPALL     3

uint_fast8_t state = INITMODE;

void codeForTask1( void * parameter )
{
  for (;;) { // loop
    Serial.print("->WELCOME");
    Serial.print("->Please enter idle temperature: ");
    if (Serial.available()) {
      _bytes = Serial.readString();
      String e0 = s.separa(_bytes, ',', 0);
      String e1 = s.separa(_bytes, ',', 1);
      if (e0 == "st") {
        if (e1 == "all") {
          printAllSensor();
        } else {
          state = IDLEMODE; // State by default
          Serial.print("Invalid command");
        }
      } else if (e0 == "it") {
        idleTemp = e1.toFloat();
        state = IDLEMODE;
      } else if (e0 == "wc") {
        wcTemp = e1.toFloat();
        state = COOLINGMODE;
      } else if (e0 == "id") {
        state = IDLEMODE;
      } else if (e0 == "cl") {
        state = STOPALL;
      } else {
        state = IDLEMODE; // State by default
        Serial.print("Invalid command");
      }
    }
    // print every second
    static unsigned long previousMillis = 0;
    if ((millis() - previousMillis) >= 1000) { // Millise
      previousMillis += millis();
      printAllSensor();
    }
  }
  vTaskDelay(10);
}

void setup() {
  Serial.begin(115200);

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

  pinMode(S1_C1, OUTPUT);
  pinMode(S1_C2, OUTPUT);
  pinMode(S2_C1, OUTPUT);
  pinMode(S2_C2, OUTPUT);
  pinMode(S3_C1, OUTPUT);
  pinMode(S3_C2, OUTPUT);
  pinMode(S4_C1, OUTPUT);
  pinMode(S4_C2, OUTPUT);

  for (uint_fast8_t i = 0; i < 4; i++) {
    // configure LED PWM functionalitites
    ledcSetup(PWM_CHANNEL[i], PWM_FREQUENCY, PWM_RESOUTION);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(GPIOPIN[i], PWM_CHANNEL[i]);
  }

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
      writeOnPump(HIGH);
      softStartPeltier();
      dataDac = 127; //half gear
      dacSendByte(dataDac);
      state = IDLEMODE;
      break;
    case IDLEMODE:
      dataDac = 255; //half gear
      dacSendByte(dataDac);
      controlTemp(idleTemp);
      break;
    case COOLINGMODE:
      dataDac = 255; //half gear
      dacSendByte(dataDac);
      controlTemp(wcTemp);
      break;
    case STOPALL:
      writeTempPel(0, 0, 0);
      writeTempPel(1, 0, 0);
      writeTempPel(2, 0, 0);
      writeTempPel(3, 0, 0);
      writeOnPump(LOW);
      dataDac = 0;
      dacSendByte(dataDac);
      break;
  }
  readAllSensors();
}

void readAllSensors() {
  readCountRpmFlow();
  readPumpRpmAndFlowMeter();
  //readSpareDigitalInput
  spareDigitalInput[0] = pcf8574.digitalRead(P0);
  spareDigitalInput[1] = pcf8574.digitalRead(P1);
  spareDigitalInput[2] = pcf8574.digitalRead(P2);
  spareDigitalInput[3] = pcf8574.digitalRead(P3);

  for (uint_fast8_t i = 0; i < 4; i++) {
    if (i == 3) {
      tempSenAp[i] = readTempSenAp(i, false); // jumper
    } else {
      tempSenAp[i] = readTempSenAp(i, true);
    }
  }
  for (uint_fast8_t i = 0; i < 9; i++) {
    tempsPel[i] = readTempPel(i);
  }
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
  Wire.endTransmission();      // stop transmitting
}

/*

*/
void readPumpRpmAndFlowMeter() {
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
    uint_fast8_t val = pcf8574.digitalRead(P6);
    if (val == HIGH) countR++;
    uint_fast8_t val1 = pcf8574.digitalRead(P7);
    if (val1 == HIGH) countF++;
  }
}

/*
  port 0...3
*/
float readTempSenAp(uint_fast8_t port, bool isNTC) {
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
float readTempPel(uint_fast8_t number) {
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
  HIGH OR LOW
*/
void writeOnPump(uint_fast8_t state) {
  pcf8574.digitalWrite(P5, state);
  if (state == HIGH) {
    pumpState = 1; //ON
  } else {
    pumpState = 0; //OFF
  }
}

/*
   number 0...3
   cmode  0 - > cooling
   cmode  1 - > heating
*/
void writeTempPel(uint_fast8_t number, uint_fast8_t cmode, uint_fast16_t power) {
  switch (number) {
    case 0:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0) {
        digitalWrite(S1_C1, HIGH);
        digitalWrite(S1_C2, LOW);
      } else if (cmode == 1) {
        digitalWrite(S1_C1, LOW);
        digitalWrite(S1_C2, HIGH);
      }
      break;
    case 1:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0) {
        digitalWrite(S2_C1, HIGH);
        digitalWrite(S2_C2, LOW);
      } else if (cmode == 1) {
        digitalWrite(S2_C1, LOW);
        digitalWrite(S2_C2, HIGH);
      }
      break;
    case 2:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0) {
        digitalWrite(S3_C1, HIGH);
        digitalWrite(S3_C2, LOW);
      } else if (cmode == 1) {
        digitalWrite(S3_C1, LOW);
        digitalWrite(S3_C2, HIGH);
      }
      break;
    case 3:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0) {
        digitalWrite(S4_C1, HIGH);
        digitalWrite(S4_C2, LOW);
      } else if (cmode == 1) {
        digitalWrite(S4_C1, LOW);
        digitalWrite(S4_C2, HIGH);
      }
      break;
  }
}
/*

*/
void softStartPeltier() {
  for (uint_fast16_t dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    writeTempPel(0, 0, dutyCycle);
    writeTempPel(1, 0, dutyCycle);
    writeTempPel(2, 0, dutyCycle);
    writeTempPel(3, 0, dutyCycle);
    dacSendByte(dutyCycle);
    delay(7);
  }
}
/*

*/
void controlTemp(float _temp) {
  if (tempsPel[8] < _temp - 1) { //OFF COOL
    writeTempPel(0, 0, 0);
    writeTempPel(1, 0, 0);
  }
  if (tempsPel[9] < _temp - 1) { //OFF COOL
    writeTempPel(2, 0, 0);
    writeTempPel(3, 0, 0);
  }
  if (tempsPel[8] > _temp + 1) { //ON COOL
    writeTempPel(0, 0, 255);
    writeTempPel(1, 0, 255);
  }
  if (tempsPel[9] > _temp + 1) { //ON COOL
    writeTempPel(2, 0, 255);
    writeTempPel(3, 0, 255);
  }
}

/*

*/
void printAllSensor() {

  for (uint_fast8_t i = 0; i < 10; i++) {
    Serial.print(tempsPel[i]);
    Serial.print(",");
  }
  Serial.print(pumpState);
  Serial.print(",");
  Serial.print(pumpRpm);
  Serial.print(",");
  Serial.print(flowLPM);
  Serial.print(",");
  for (uint_fast8_t i = 0; i < 3; i++) {
    Serial.print(spareDigitalInput[i]);
    Serial.print(",");
  }
  for (uint_fast8_t i = 0; i < 4; i++) {
    Serial.print(tempSenAp[i]);
    Serial.print(",");
  }
  Serial.print(dataDac);
}
