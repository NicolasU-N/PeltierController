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

uint16_t PWM_FREQUENCY = 1000; // this variable is used to define the time period
uint8_t PWM_RESOUTION = 8; // this will define the resolution of the signal which is 8 in this case
uint8_t PWM_CHANNEL[] = {0, 1, 2, 3}; // this variable is used to select the channel number
uint8_t GPIOPIN[] = {SRC1, SRC2, SRC3, SRC4}; // GPIO to which we want to attach this channel signal
//uint_fast16_t dutyCycle = 0; // it will define the width of signal or also the one time


void setup() {
  Serial.begin(115200);

  pinMode(S1_C1, OUTPUT);
  pinMode(S1_C2, OUTPUT);
  pinMode(S2_C1, OUTPUT);
  pinMode(S2_C2, OUTPUT);
  pinMode(S3_C1, OUTPUT);
  pinMode(S3_C2, OUTPUT);
  pinMode(S4_C1, OUTPUT);
  pinMode(S4_C2, OUTPUT);

  writeTempPel(0, 2, 0); // OFF
  writeTempPel(1, 2, 0);
  writeTempPel(2, 2, 0);
  writeTempPel(3, 2, 0);

  Serial.print("->WELCOME");
}

void loop() {
  writeTempPel(0, 0, 127);
  delay(1000);
  writeTempPel(1, 0, 127);
  delay(1000);
  writeTempPel(2, 0, 127);
  delay(1000);
  writeTempPel(3, 0, 127);
  delay(2000);
  writeTempPel(0, 2, 0); // OFF
  delay(1000);
  writeTempPel(1, 2, 0);
  delay(1000);
  writeTempPel(2, 2, 0);
  delay(1000);
  writeTempPel(3, 2, 0);
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
