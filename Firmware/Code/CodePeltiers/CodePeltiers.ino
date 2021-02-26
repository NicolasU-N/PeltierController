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
#define STARTMODE   1
#define IDLEMODE    2
#define COOLINGMODE 3

uint_fast8_t state = INITMODE;

void codeForTask1( void * parameter )
{
  for (;;) { // loop

    Serial.write("->WELCOME");
    Serial.write("->Please enter idle temperature: ");

    if (Serial.available()) {
      _bytes = Serial.readString();
      String e0 = s.separa(_bytes, ',', 0);
      String e1 = s.separa(_bytes, ',', 1);
      if (e0 == "st") {
        if (e1 == "all") {
          char* buf0 = "" ;
          char* buf1 = "" ;
          char* buf2 = "" ;
          char* buf3 = "" ;
          char* buf4 = "" ;
          char* buf5 = "" ;
          char* buf6 = "" ;
          char* buf7 = "" ;
          char* buf8 = "" ;
          char* buf9 = "" ;
          char* buf10 = "" ;
          char* buf11 = "" ;
          char* buf12 = "" ;
          char* buf13 = "" ;
          char* buf14 = "" ;
          char* buf15 = "" ;
          char* buf16 = "" ;
          char* buf17 = "" ;
          char* buf18 = "" ;
          char* buf19 = "" ;
          char* buf20 = "" ;
          dtostrf(tempsPel[0], 10, 3, buf0); //Peltiers
          dtostrf(tempsPel[1], 10, 3, buf1); //Peltiers
          dtostrf(tempsPel[2], 10, 3, buf2); //Peltiers
          dtostrf(tempsPel[3], 10, 3, buf3); //Peltiers
          dtostrf(tempsPel[4], 10, 3, buf4); //Peltiers
          dtostrf(tempsPel[5], 10, 3, buf5); //Peltiers
          dtostrf(tempsPel[6], 10, 3, buf6); //Peltiers
          dtostrf(tempsPel[7], 10, 3, buf7); //Peltiers
          dtostrf(tempsPel[8], 10, 3, buf8); //Peltiers
          dtostrf(tempsPel[9], 10, 3, buf9); //Peltiers
          dtostrf(pumpState, 10, 3, buf10);
          dtostrf(pumpRpm, 10, 3, buf11);
          dtostrf(flowLPM, 10, 3, buf12);
          dtostrf(spareDigitalInput[0], 10, 3, buf13);
          dtostrf(spareDigitalInput[1], 10, 3, buf14);
          dtostrf(spareDigitalInput[2], 10, 3, buf15);
          dtostrf(tempSenAp[0], 10, 3, buf16);
          dtostrf(tempSenAp[1], 10, 3, buf17);
          dtostrf(tempSenAp[2], 10, 3, buf18);
          dtostrf(tempSenAp[3], 10, 3, buf19);
          dtostrf((float)dataDac, 10, 3, buf20);

          char buffer[255] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0, buf1, buf2, buf3, buf4, buf5, buf6, buf7, buf8, buf9, buf10, buf11, buf12, buf13, buf14, buf15, buf16, buf17, buf18, buf19, buf20);
          Serial.write(buffer);
        } else if (e1 == "out") {
          char* buf0 = "" ;
          char* buf1 = "" ;
          dtostrf(tempsPel[8], 10, 3, buf0); //Peltiers
          dtostrf(tempsPel[9], 10, 3, buf1); //Peltiers
          char buffer[50] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s,%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0, buf1);
          Serial.write(buffer);

        } else if (e1 == "pel") {
          char* buf0 = "" ;
          char* buf1 = "" ;
          char* buf2 = "" ;
          char* buf3 = "" ;
          char* buf4 = "" ;
          char* buf5 = "" ;
          char* buf6 = "" ;
          char* buf7 = "" ;
          char* buf8 = "" ;
          char* buf9 = "" ;
          dtostrf(tempsPel[0], 10, 3, buf0); //Peltiers
          dtostrf(tempsPel[1], 10, 3, buf1); //Peltiers
          dtostrf(tempsPel[2], 10, 3, buf2); //Peltiers
          dtostrf(tempsPel[3], 10, 3, buf3); //Peltiers
          dtostrf(tempsPel[4], 10, 3, buf4); //Peltiers
          dtostrf(tempsPel[5], 10, 3, buf5); //Peltiers
          dtostrf(tempsPel[6], 10, 3, buf6); //Peltiers
          dtostrf(tempsPel[7], 10, 3, buf7); //Peltiers
          dtostrf(tempsPel[8], 10, 3, buf8); //Peltiers
          dtostrf(tempsPel[9], 10, 3, buf9); //Peltiers
          char buffer[100] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s,%s,%s,%s,%s,%s,%s,%s,%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0, buf1, buf2, buf3, buf4, buf5, buf6, buf7, buf8, buf9);
          Serial.write(buffer);
        } else if (e1 == "pump") {
          char* buf0 = "" ;
          char* buf1 = "" ;
          dtostrf(pumpState, 10, 3, buf0); //Peltiers
          dtostrf(pumpRpm, 10, 3, buf1); //Peltiers
          char buffer[100] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s,%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0, buf1);
          Serial.write(buffer);
        } else if (e1 == "chiller") {
          char* buf0 = "" ;
          dtostrf((float)dataDac, 10, 3, buf0); //Peltiers
          char buffer[100] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0);
          Serial.write(buffer);
        } else if (e1 == "flowmeter") {
          char* buf0 = "" ;
          dtostrf(flowLPM, 10, 3, buf0); //Peltiers
          char buffer[100] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0);
          Serial.write(buffer);
        } else if (e1 == "coolantTemp") {
          char* buf0 = "" ;
          char* buf1 = "" ;
          char* buf2 = "" ;
          dtostrf(tempSenAp[0], 10, 3, buf0); //Peltiers
          dtostrf(tempSenAp[1], 10, 3, buf1); //Peltiers
          dtostrf(tempSenAp[2], 10, 3, buf2); //Peltiers
          char buffer[100] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s,%s,%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0, buf1, buf2);
          Serial.write(buffer);
        } else if (e1 == "spareAIn") {
          char* buf0 = "" ;
          dtostrf(tempSenAp[3], 10, 3, buf0); //Peltiers
          char buffer[100] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0);
          Serial.write(buffer);
        }
        else if (e1 == "spareDIn") {
          char* buf0 = "" ;
          char* buf1 = "" ;
          char* buf2 = "" ;
          dtostrf(spareDigitalInput[0], 10, 3, buf0);
          dtostrf(spareDigitalInput[1], 10, 3, buf1);
          dtostrf(spareDigitalInput[2], 10, 3, buf2);
          char buffer[100] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
          char* formato = "status:%s,%s,%s"; //Cadena con la mascara a convertir
          sprintf(buffer, formato, buf0);
          Serial.write(buffer);
        }
      } else if (e0 == "it") {
        idleTemp = e1.toFloat();
        state = IDLEMODE;
      } else if (e0 == "wc") {
        wcTemp = e1.toFloat();
        state = STARTMODE;
      } else if (e0 == "id") {
        wcTemp = e1.toFloat();
        state = STARTMODE;
      } else {
        state = IDLEMODE; // State by default
        Serial.write("Invalid command");
      }
    }

    // print every second
    static unsigned long previousMillis = 0;
    if ((millis() - previousMillis) >= 1000) { // Millise
      previousMillis += millis();
      char* buf0 = "" ;
      char* buf1 = "" ;
      char* buf2 = "" ;
      char* buf3 = "" ;
      char* buf4 = "" ;
      char* buf5 = "" ;
      char* buf6 = "" ;
      char* buf7 = "" ;
      char* buf8 = "" ;
      char* buf9 = "" ;
      char* buf10 = "" ;
      char* buf11 = "" ;
      char* buf12 = "" ;
      char* buf13 = "" ;
      char* buf14 = "" ;
      char* buf15 = "" ;
      char* buf16 = "" ;
      char* buf17 = "" ;
      char* buf18 = "" ;
      char* buf19 = "" ;
      char* buf20 = "" ;
      dtostrf(tempsPel[0], 10, 3, buf0); //Peltiers
      dtostrf(tempsPel[1], 10, 3, buf1); //Peltiers
      dtostrf(tempsPel[2], 10, 3, buf2); //Peltiers
      dtostrf(tempsPel[3], 10, 3, buf3); //Peltiers
      dtostrf(tempsPel[4], 10, 3, buf4); //Peltiers
      dtostrf(tempsPel[5], 10, 3, buf5); //Peltiers
      dtostrf(tempsPel[6], 10, 3, buf6); //Peltiers
      dtostrf(tempsPel[7], 10, 3, buf7); //Peltiers
      dtostrf(tempsPel[8], 10, 3, buf8); //Peltiers
      dtostrf(tempsPel[9], 10, 3, buf9); //Peltiers
      dtostrf(pumpState, 10, 3, buf10);
      dtostrf(pumpRpm, 10, 3, buf11);
      dtostrf(flowLPM, 10, 3, buf12);
      dtostrf(spareDigitalInput[0], 10, 3, buf13);
      dtostrf(spareDigitalInput[1], 10, 3, buf14);
      dtostrf(spareDigitalInput[2], 10, 3, buf15);
      dtostrf(tempSenAp[0], 10, 3, buf16);
      dtostrf(tempSenAp[1], 10, 3, buf17);
      dtostrf(tempSenAp[2], 10, 3, buf18);
      dtostrf(tempSenAp[3], 10, 3, buf19);
      dtostrf((float)dataDac, 10, 3, buf20);

      char buffer[255] = " "; //Buffer de la cadena donde se devuelve todo, número formateado y cadena concatenada
      char* formato = "status:%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s"; //Cadena con la mascara a convertir
      sprintf(buffer, formato, buf0, buf1, buf2, buf3, buf4, buf5, buf6, buf7, buf8, buf9, buf10, buf11, buf12, buf13, buf14, buf15, buf16, buf17, buf18, buf19, buf20);
      Serial.write(buffer);
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
      softStartPeltier();
      dataDac = 127;
      dacSendByte(dataDac);
      state = IDLEMODE;
      break;
    //    case STARTMODE:
    //      // statements
    //      break;
    case IDLEMODE:
      // statements
      break;
    case COOLINGMODE:
      // statements
      break;
  }

  //CONTROL TEMP

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
