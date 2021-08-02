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
#include "Adafruit_ADS1015.h"
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

#define ADDR_DAC  0x0D
#define ADDR_PCF  0x38
#define ADDR_ADS0 0x48
#define ADDR_ADS1 0x49
#define ADDR_ADS2 0x4A
#define ADDR_ADS3 0x4B

TaskHandle_t Task1;

// Set i2c HEX address
PCF8574 pcf8574(&Wire, 0x38);
Adafruit_ADS1115 ads0;
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


float spareDigitalInput[4]; //uint_fast8_t
float tempSenAp[4];
uint8_t dataDac = 127;

float r1 = 10000; // RES PULL DOWN
float TH1 = 5000; // 5K
float TH2 = 15000; // 15K
float TH3 = 10000; // 15K

float TempN = 25; // temp nom

float bcoe1 = 3865.24; //5K
float bcoe2 = 3876.00; //15K
float bcoe3 = 3368.06; //15K

//float c1 = 1.283620738e-03, c2 = 2.364048201e-04, c3 = 0.9207627775e-07; //5k 3176
//float c4 = 1.035279821e-03, c5 = 2.338110730e-04, c6 = 0.7924168116e-07; //15k 3193
//https://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm

uint16_t PWM_FREQUENCY = 1000; // this variable is used to define the time period
uint8_t PWM_RESOUTION = 8; // this will define the resolution of the signal which is 8 in this case
uint8_t PWM_CHANNEL[] = {0, 1, 2, 3}; // this variable is used to select the channel number
uint8_t GPIOPIN[] = {SRC1, SRC2, SRC3, SRC4}; // GPIO to which we want to attach this channel signal
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

uint8_t state = INITMODE;

void codeForTask1( void * parameter )
{
  for (;;) { // loop
    dacSendByte(dataDac);

    // print every second
    static unsigned long previousMillis = 0;
    if ((millis() - previousMillis) >= 500) { // Millise
      readAllSensors();
      printAllSensor();
      previousMillis += 500;
    }
  }
  vTaskDelay(2);
}


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

  for (uint8_t i = 0; i < 4; i++) {
    // configure LED PWM functionalitites
    ledcSetup(PWM_CHANNEL[i], PWM_FREQUENCY, PWM_RESOUTION);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(GPIOPIN[i], PWM_CHANNEL[i]);
  }
  pumpState = 0;
  pcf8574.digitalWrite(5, 0);
  delay(300);
  pcf8574.digitalWrite(5, 1);
  delay(300);
  pcf8574.digitalWrite(5, 0);

  xTaskCreatePinnedToCore(
    codeForTask1,
    "Send dac",
    1000,
    NULL,
    1,
    &Task1,
    1);



  Serial.print("->WELCOME");
  Serial.println("->Please enter idle temperature: ");
}

void loop() {
  switch (state) {
    case INITMODE:
      writeOnPump(0); // On pump
      //pumpState = 0;
      //pcf8574.digitalWrite(5, 0);
      softStartPeltier();
      dataDac = 127; //half gear
      //dacSendByte(dataDac);
      state = IDLEMODE;
      break;
    case IDLEMODE:
      //dacSendByte(dataDac);
      controlTemp(idleTemp);
      break;
    case COOLINGMODE:
      //dataDac = 255; //half gear
      //dacSendByte(dataDac);
      //if ((tempsPel[6] <= wcTemp - 2) and (tempsPel[6] >= wcTemp + 2)) {
      //  state = IDLEMODE;
      //} else {
      controlTemp(wcTemp);
      //}
      break;
    case STOPALL:
      writeTempPel(0, 0, 0);
      writeTempPel(1, 0, 0);
      writeTempPel(2, 0, 0);
      writeTempPel(3, 0, 0);
      writeOnPump(1);
      dataDac = 0;
      break;
  }

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
      dataDac = 127; //half gear
      writeOnPump(0); // ON PUMP
      idleTemp = e1.toFloat();
      Serial.println("IDLEMODE");
      Serial.println(idleTemp);
      state = IDLEMODE;
    } else if (e0 == "wc") {
      dataDac = 255; //full
      writeOnPump(0);
      wcTemp = e1.toFloat();
      Serial.println("WC");
      Serial.println(wcTemp);
      state = COOLINGMODE;
    } else if (e0 == "id") {
      dataDac = 127; //half gear
      writeOnPump(0); // ON PUMP
      writeOnPump(0);
      Serial.println("IDLEMODE");
      state = IDLEMODE;
    } else if (e0 == "cl") {
      writeOnPump(1);
      Serial.println("STOP");
      state = STOPALL;
    } /////////////////////////////////////////////////////////////////////////////////////////// TEST

    else if (e0 == "t1") {
      tempsPel[0] = e1.toFloat();
      Serial.print("T1,");
      Serial.println(tempsPel[0]);
    } else if (e0 == "t2") {
      tempsPel[1] = e1.toFloat();
      Serial.print("T2,");
      Serial.println(tempsPel[1]);
    } else if (e0 == "t3") {
      tempsPel[2] = e1.toFloat();
      Serial.print("T3,");
      Serial.println(tempsPel[2]);
    } else if (e0 == "t4") {
      tempsPel[3] = e1.toFloat();
      Serial.print("T4,");
      Serial.println(tempsPel[3]);
    } else if (e0 == "t5") {
      tempsPel[4] = e1.toFloat();
      Serial.print("T5,");
      Serial.println(tempsPel[4]);
    } else if (e0 == "t6") {
      tempsPel[5] = e1.toFloat();
      Serial.print("T6,");
      Serial.println(tempsPel[5]);
    } else if (e0 == "t7") {
      tempsPel[6] = e1.toFloat();
      Serial.print("T7,");
      Serial.println(tempsPel[6]);
    } else if (e0 == "t8") {
      tempsPel[7] = e1.toFloat();
      Serial.print("T8,");
      Serial.println(tempsPel[7]);
    } else if (e0 == "t9") {
      tempsPel[8] = e1.toFloat();
      Serial.print("T9,");
      Serial.println(tempsPel[8]);
    } else if (e0 == "t10") {
      tempsPel[9] = e1.toFloat();
      Serial.print("T10,");
      Serial.println(tempsPel[9]);
    }

    else {
      state = IDLEMODE; // State by default
      Serial.print("Invalid command");
    }
  }

}

/*

*/
void controlTemp(float _temp) {
  // ON/OFF 1
  if (tempsPel[8] > _temp + 1) { //OFF COOL
    writeTempPel(0, 0, 127);
    writeTempPel(1, 0, 127);
    //Serial.println("----Writetemppel0 1 0");
  }

  if (tempsPel[8] < _temp - 1) { //OFF COOL
    writeTempPel(0, 0, 0);
    writeTempPel(1, 0, 0);
    //Serial.println("----Writetemppel0 1 0");
  }

  // ON/OFF 2
  if (tempsPel[9] > _temp + 1) { //OFF COOL
    writeTempPel(2, 0, 127);
    writeTempPel(3, 0, 127);
    //Serial.println("----Writetemppel0 1 0");
  }

  if (tempsPel[9] < _temp - 1) { //OFF COOL
    writeTempPel(2, 0, 0);
    writeTempPel(3, 0, 0);
    //Serial.println("----Writetemppel0 1 0");
  }

  //standby
  if ((tempsPel[8] >= _temp - 2) and (tempsPel[8] <= _temp + 2)) { // Está en el rango
    writeTempPel(0, 0, 0);
    writeTempPel(1, 0, 0);
  }

  if ((tempsPel[9] >= _temp - 2) and (tempsPel[9] <= _temp + 2)) { // Está en el rango
    writeTempPel(2, 0, 0);
    writeTempPel(3, 0, 0);
  }

  // HOT TEMPERATURE CONTROL
  if (tempsPel[0] >= 80 or tempsPel[1] >= 80) { //OFF COOL
    writeTempPel(0, 2, 0);
    //Serial.println("----Writetemppel0 1 0");
  }
  if (tempsPel[2] >= 80 or tempsPel[3] >= 80) { //ON COOL
    writeTempPel(1, 2, 0);
    //Serial.println("----Writetemppel0 1 127");
  }
  if (tempsPel[4] >= 80 or tempsPel[5] >= 80) { //OFF COOL
    writeTempPel(2, 2, 0);
    //Serial.println("----Writetemppel0 1 0");
  }
  if (tempsPel[6] >= 80 or tempsPel[7] >= 80) { //ON COOL
    writeTempPel(3, 2, 0);
    //Serial.println("----Writetemppel0 1 127");
  }
}

void readAllSensors() {
  readCountRpmFlow();
  readPumpRpmAndFlowMeter();
  //readSpareDigitalInput
  spareDigitalInput[0] = (float)pcf8574.digitalRead(0);
  spareDigitalInput[1] = (float)pcf8574.digitalRead(1);
  spareDigitalInput[2] = (float)pcf8574.digitalRead(2);
  spareDigitalInput[3] = (float)pcf8574.digitalRead(3);

  for (uint8_t i = 0; i < 4; i++) {
    if (i == 3) {
      tempSenAp[i] = readTempSenAp(i, false); // jumper
    } else {
      tempSenAp[i] = readTempSenAp(i, true);
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////
  /*
    for (uint8_t i = 0; i < 10; i++) {
    tempsPel[i] = readTempPel(i);
    }
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
  Wire.endTransmission();      // stop transmitting
}

/*

*/
void readPumpRpmAndFlowMeter() {
  static unsigned long previousMillis = 0;
  if ((millis() - previousMillis) >= 1000) { // Millise
    pumpRpm = countR * 30; // 2 pulses per turn (60/2)
    countR = 0;
    flowLPM = countF * 0.283; // LPM = 0.283 x Hz
    countF = 0;
    previousMillis += 1000;
  }
}

/*

*/
void readCountRpmFlow() {
  static unsigned long previousMillis = 0;
  if ((millis() - previousMillis) >= 2) { // Millise
    uint8_t val = pcf8574.digitalRead(6);
    uint8_t val1 = pcf8574.digitalRead(7);
    if (val == 1) {
      countR++;
    }
    if (val1 == 1) {
      countF++;
    }
    previousMillis += 2;
  }
}

/*
  port 0...3
*/
float readTempSenAp(uint8_t port, bool isNTC) {
  if (isNTC) {
    float vout = (float)ads3.readADC_SingleEnded(port); //0...32767 resolution
    float R_NTC = (32768 / vout) - 1; //calculating the resistance of the thermistor
    R_NTC = r1 / R_NTC;
    float Temp_K = (R_NTC / TH3);
    Temp_K = log(Temp_K);
    Temp_K /= bcoe3;
    Temp_K += 1.0 / (TempN + 273.15);
    Temp_K = 1.0 / Temp_K;
    Temp_K -= 273.15;
    Temp_K -= 22.11;

    return Temp_K;
  } else {
    return (float)ads3.readADC_SingleEnded(port);
  }
}

/*
  0...9
*/
float readTempPel(uint8_t number) {
  float vout = 0;
  float R_NTC = 0;
  float Temp_K  = 0;  

  if (number >= 0 and number <= 3) { //T1-T4
    vout = (float)ads0.readADC_SingleEnded(number ); // * (3.3 / 32767); //0...32767 resolutio
    R_NTC = (32768 / vout) - 1; //calculating the resistance of the thermistor
    R_NTC = r1 / R_NTC; //calculating the resistance of the thermistor
    Temp_K = (R_NTC / TH1); //Temperature in Kelvin B_param and T0 = 25C TO K
    Temp_K = log(Temp_K);
    Temp_K /= bcoe1;
    Temp_K += 1.0 / (TempN + 273.15);
    Temp_K = 1.0 / Temp_K;
    Temp_K -= 273.15;
    Temp_K += 8.11;
  } else if (number > 3  and number <= 7) { //T5-T8
    vout = (float)ads1.readADC_SingleEnded(number - 4);// * (3.3 / 32767); //0...32767 resolution
    R_NTC = (32768 / vout) - 1; //calculating the resistance of the thermistor
    R_NTC = r1 / R_NTC; //calculating the resistance of the thermistor
    Temp_K = (R_NTC / TH1); //Temperature in Kelvin B_param and T0 = 25C TO K
    Temp_K = log(Temp_K);
    Temp_K /= bcoe1;
    Temp_K += 1.0 / (TempN + 273.15);
    Temp_K = 1.0 / Temp_K;
    Temp_K -= 273.15;
    Temp_K += 8.11;
  } else if (number > 7) { //T9-T10
    vout = (float)ads2.readADC_SingleEnded(number - 8);// * (3.3 / 32767); //0...32767 resolution
    R_NTC = (32768 / vout) - 1; //calculating the resistance of the thermistor
    R_NTC = r1 / R_NTC; //calculating the resistance of the thermistor
    Temp_K = (R_NTC / TH2); //Temperature in Kelvin B_param and T0 = 25C TO K
    Temp_K = log(Temp_K);
    Temp_K /= bcoe2;
    Temp_K += 1.0 / (TempN + 273.15);
    Temp_K = 1.0 / Temp_K;
    Temp_K -= 273.15;
    Temp_K -= 37.44;
  }

  //float Temp_C = Temp_K - 273.15; //converting into Celsius
  return Temp_K;//Temp_C;
}

/*
  HIGH OR LOW
*/
void writeOnPump(uint8_t state) {
  pcf8574.digitalWrite(5, state);
  if (state == 1) {
    pumpState = 1; //OFF
  } else {
    pumpState = 0; //ON
  }
}

/*
   number 0...3
   cmode  0 - > cooling
   cmode  1 - > heating
   cmode  2 - > STOP
*/
void writeTempPel(uint8_t number, uint8_t cmode, uint16_t power) {
  switch (number) {
    case 0:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0) {
        digitalWrite(S1_C1, HIGH);
        digitalWrite(S1_C2, LOW);
      } else if (cmode == 1) {
        digitalWrite(S1_C1, LOW);
        digitalWrite(S1_C2, HIGH);
      } else if (cmode == 2) {
        digitalWrite(S1_C1, LOW);
        digitalWrite(S1_C2, LOW);
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
      } else if (cmode == 2) {
        digitalWrite(S2_C1, LOW);
        digitalWrite(S2_C2, LOW);
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
      } else if (cmode == 2) {
        digitalWrite(S3_C1, LOW);
        digitalWrite(S3_C2, LOW);
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
      } else if (cmode == 2) {
        digitalWrite(S4_C1, LOW);
        digitalWrite(S4_C2, LOW);
      }
      break;
  }
}
/*

*/
void softStartPeltier() {
  for (uint16_t dutyCycle = 0; dutyCycle <= 127; dutyCycle++) {
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
void printAllSensor() {
  for (uint8_t i = 0; i < 10; i++) {
    Serial.print(tempsPel[i]);
    Serial.print(",");
  }
  Serial.print(pumpState);
  Serial.print(",");
  Serial.print(pumpRpm);
  Serial.print(",");
  Serial.print(flowLPM);
  Serial.print(",");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(spareDigitalInput[i]);
    Serial.print(",");
  }
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(tempSenAp[i]);
    Serial.print(",");
  }
  Serial.println(dataDac);
}
