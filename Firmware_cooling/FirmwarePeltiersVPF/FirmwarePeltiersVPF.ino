/*
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
#include <Separador.h>
#include "MYADS.h"
#include "SingleEMAFilterLib.h"


#define SRC1 16
#define S1_C1 23
#define S1_C2 25
#define SRC2 17
#define S2_C1 11
#define S2_C2 12
#define SRC3 18
#define S3_C1 32
#define S3_C2 33
#define SRC4 19
#define S4_C1 13
#define S4_C2 4

#define ADDR_DAC 0x0D
#define ADDR_PCF 0x38

// Set i2c HEX address
PCF8574 pcf8574(&Wire, 0x38);

SingleEMAFilter<float> filter1(0.4); // 0.2
SingleEMAFilter<float> filter2(0.4); // 0.2
SingleEMAFilter<float> filter3(0.4); // 0.2
SingleEMAFilter<float> filter4(0.4); // 0.2
SingleEMAFilter<float> filter5(0.4); // 0.2
SingleEMAFilter<float> filter6(0.4); // 0.2

// Filters for coolant temperature measurement
SingleEMAFilter<float> filter7(0.4);  // 0.2
SingleEMAFilter<float> filter8(0.4);  // 0.2
SingleEMAFilter<float> filter9(0.4);  // 0.2
SingleEMAFilter<float> filter10(0.4); // 0.2

MYADS Myads; //initialize ads

String _bytes;

Separador s;

volatile int countR = 0;
volatile int countF = 0;

uint8_t pumpState = 0;
int pumpRpm = 0; //uint_fast16_t
float flowLPM = 0;

//----------------------------------------------------------- PELTIERS
// Temps peltiers 0...7 Heat 8..9 Cool
float vin = 3260.0;
float tempsPel[6];

float R1 = 10000;  //5K sensor resistor    5000
float R3 = 10000; //15K sensor resistor   15000
float R4 = 10000; //10K sensor resistor   10000
//-----------------------------------------------------------

//----------------------------------------------------------- COOLANT SENSORS
float tempSenCoolant[4];
//-----------------------------------------------------------

//----------------------------------------------------------- DETECT RISING EDGES
#define PUMP 0 //index
#define FLOWMETER 1

//Pin of pcf8574 corresponding to the input
uint8_t button[2] = {
  6, // PUMP
  7, // FLOW METER
};

uint8_t button_state[2]; // 0 -> RMP 1 -> FLOW METER
//-----------------------------------------------------------

//float spareDigitalInput[4]; //uint_fast8_t ----------------------------------- DEBUG SPARE DIGITAL INPUT

uint8_t dataDac = 127;

uint16_t PWM_FREQUENCY = 1000;                // this variable is used to define the time period
uint8_t PWM_RESOUTION = 8;                    // this will define the resolution of the signal which is 8 in this case
uint8_t PWM_CHANNEL[] = {0, 1, 2, 3};         // this variable is used to select the channel number
uint8_t GPIOPIN[] = {SRC1, SRC2, SRC3, SRC4}; // GPIO to which we want to attach this channel signal
//uint_fast16_t dutyCycle = 0; // it will define the width of signal or also the one time

float idleTemp = 10; //default
float wcTemp = 6;    //default

////////////////////////
//       STATES       //
////////////////////////
#define INITMODE 0
#define IDLEMODE 1
#define COOLINGMODE 2
#define STOPALL 3

uint8_t state = INITMODE;

TaskHandle_t task1_handle;

void codeForTask1(void *parameter)
{
  for (;;)
  { // loop
    //if (state != STOPALL)
    //{
    //  dacSendByte(dataDac);
    //}
    //else
    //{
    //  dacSendByte(0);
    //}
    delay(15);
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void setup()
{
  Serial.begin(115200);

  pcf8574.pinMode(P1, INPUT);
  pcf8574.pinMode(P2, INPUT);
  pcf8574.pinMode(P3, INPUT);
  pcf8574.pinMode(P5, OUTPUT, LOW);
  pcf8574.pinMode(P7, INPUT);
  pcf8574.pinMode(P6, INPUT);

  Wire.begin();
  pcf8574.begin();

  pinMode(S1_C1, OUTPUT);
  pinMode(S1_C2, OUTPUT);
  pinMode(S2_C1, OUTPUT);
  pinMode(S2_C2, OUTPUT);
  pinMode(S3_C1, OUTPUT);
  pinMode(S3_C2, OUTPUT);
  pinMode(S4_C1, OUTPUT);
  pinMode(S4_C2, OUTPUT);

  for (uint8_t i = 0; i < 4; i++)
  {
    // configure LED PWM functionalitites
    ledcSetup(PWM_CHANNEL[i], PWM_FREQUENCY, PWM_RESOUTION);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(GPIOPIN[i], PWM_CHANNEL[i]);
  }

  xTaskCreatePinnedToCore(
    codeForTask1,
    "Send data to dac",
    4096,
    NULL,
    0,
    NULL, //&task1_handle
    0);

  Serial.println("--> WELCOME!");
  Serial.println("--> Please enter idle temperature: ");

  pcf8574.digitalWrite(5, 0);
  delay(300);
  pcf8574.digitalWrite(5, 1);
  delay(300);
  pcf8574.digitalWrite(5, 0);
}

void loop()
{

  // READ PUMP FLOW AND RPM SENSOR
  readPumpRpmAndFlowMeter();

  static unsigned long previousMillis2 = 0;
  if ((millis() - previousMillis2) >= 200)
  { // Millise
    readAllSensors();
    previousMillis2 += 200;
  }

  /*
    static unsigned long previousMillis3 = 0;
    if ((millis() - previousMillis3) >= 500)
    { // Millise
      printAllSensor();
      previousMillis3 += 500;
    }
  */

  switch (state)
  {
    case INITMODE:
      writeOnPump(0); // On pump
      softStartPeltier();
      state = IDLEMODE;
      break;
    case IDLEMODE:
      controlTemp(idleTemp);
      break;
    case COOLINGMODE:
      //if ((tempsPel[6] <= wcTemp - 2) and (tempsPel[6] >= wcTemp + 2)) {
      //  state = IDLEMODE;
      //} else {
      controlTemp(wcTemp);
      //}
      break;

    // -------------------------------------------------- MAY BE OPTIONAL
    case STOPALL:
      writeTempPel(0, 0, 0);
      writeTempPel(1, 0, 0);
      writeTempPel(2, 0, 0);
      writeTempPel(3, 0, 0);
      writeOnPump(1);
      dacSendByte(0);
      break;
  }

  if (Serial.available())
  {
    _bytes = Serial.readString();
    String e0 = s.separa(_bytes, ',', 0);
    String e1 = s.separa(_bytes, ',', 1);

    /*
      if (e0 == "st")
      {
      if (e1 == "all")
      {
        printAllSensor();
      }
      else
      {
        state = IDLEMODE; // State by default
        Serial.print("Invalid command");
      }
      }
    */

    if (e0 == "it")
    {
      dataDac = 127;  //half gear
      writeOnPump(0); // ON PUMP
      idleTemp = e1.toFloat();
      Serial.println("IDLEMODE");
      Serial.println(idleTemp);
      state = IDLEMODE;
    }
    else if (e0 == "wc")
    {
      dataDac = 255; //full
      writeOnPump(0);
      wcTemp = e1.toFloat();
      Serial.println("WC");
      Serial.println(wcTemp);
      state = COOLINGMODE;
    }
    else if (e0 == "id")
    {
      dataDac = 127;  //half gear
      writeOnPump(0); // ON PUMP
      Serial.println("IDLEMODE");
      state = IDLEMODE;
    }
    else if (e0 == "cl")
    {
      writeTempPel(0, 0, 0);
      writeTempPel(1, 0, 0);
      writeTempPel(2, 0, 0);
      writeTempPel(3, 0, 0);
      writeOnPump(1);
      dacSendByte(0);
      Serial.println("STOP");
      state = STOPALL;
    }
    else
    {
      state = IDLEMODE; // State by default
      Serial.print("Invalid command");
    }
  }
}

void readAllSensors()
{
  /*
    //readSpareDigitalInput
    spareDigitalInput[0] = (float)pcf8574.digitalRead(0);
    spareDigitalInput[1] = (float)pcf8574.digitalRead(1);
    spareDigitalInput[2] = (float)pcf8574.digitalRead(2);
    spareDigitalInput[3] = (float)pcf8574.digitalRead(3);
  */
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  readTempPel();

  for (uint8_t i = 0; i < 3; i++) //0...2 sensor inputs senap008b
  {
    readTempCoolant(i);
  }

  // --------------------------------------------- Print values
  for (uint8_t i = 0; i < 6; i++)
  {
    Serial.print(tempsPel[i]);
    Serial.print(",");
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    Serial.print(tempSenCoolant[0]);
    Serial.print(",");
  }

  Serial.print(pumpState);
  Serial.print(",");
  Serial.print(pumpRpm);
  Serial.print(",");
  Serial.print(flowLPM);
  Serial.print(",");

  /*
    for (uint8_t i = 0; i < 4; i++) {
    Serial.print(spareDigitalInput[i]);
    Serial.print(",");
    }
    for (uint8_t i = 0; i < 4; i++) {
    Serial.print(tempSenAp[i]);
    Serial.print(",");
    }
  */

  Serial.println(dataDac);
}



/*
  @brief Cooling temperature control function
  @param float _temp target temperature
*/
void controlTemp(float _temp)
{
  // ON/OFF 1
  if (tempsPel[4] > _temp + 1)
  { //OFF COOL
    writeTempPel(0, 0, 127);
    writeTempPel(1, 0, 127);
  }

  if (tempsPel[4] < _temp - 1)
  { //OFF COOL
    writeTempPel(0, 0, 0);
    writeTempPel(1, 0, 0);
  }

  // ON/OFF 2
  if (tempsPel[5] > _temp + 1)
  { //OFF COOL
    writeTempPel(2, 0, 127);
    writeTempPel(3, 0, 127);
  }

  if (tempsPel[5] < _temp - 1)
  { //OFF COOL
    writeTempPel(2, 0, 0);
    writeTempPel(3, 0, 0);
  }

  //standby
  if ((tempsPel[4] >= _temp - 2) and (tempsPel[4] <= _temp + 2))
  { // Está en el rango
    writeTempPel(0, 0, 0);
    writeTempPel(1, 0, 0);
  }

  if ((tempsPel[5] >= _temp - 2) and (tempsPel[5] <= _temp + 2))
  { // Está en el rango
    writeTempPel(2, 0, 0);
    writeTempPel(3, 0, 0);
  }

  // HOT TEMPERATURE CONTROL
  if (tempsPel[0] >= 80 )
  { //OFF COOL
    writeTempPel(0, 2, 0);
  }
  if (tempsPel[1] >= 80) {
    writeTempPel(1, 2, 0);
  }

  if (tempsPel[2] >= 80 )
  { //OFF COOL
    writeTempPel(2, 2, 0);
  }

  if (tempsPel[3] >= 80) {
    //OFF COOL
    writeTempPel(3, 2, 0); // ------------------------------------------------------------------------------ Check thermistor connection
  }


  /*
    if (tempsPel[4] >= 80 or tempsPel[5] >= 80)
    { //OFF COOL

    }
  */
}

/*
  @brief Read flow sensor and coolant pump encoder through pcf8574
*/
uint8_t flancoSubida(int btn)
{
  uint8_t valor_nuevo = pcf8574.digitalRead(button[btn]);
  uint8_t result = button_state[btn] != valor_nuevo && valor_nuevo == 1;
  button_state[btn] = valor_nuevo;
  return result;
}

/*
  @brief Read flow sensor and coolant pump encoder through pcf8574
*/
void readPumpRpmAndFlowMeter()
{
  if (flancoSubida(PUMP)) // PUMP
  {
    countR++; // RPM counter
  }

  if (flancoSubida(FLOWMETER)) // FM18T
  {
    countF++; // Flowmeter counter
  }

  static unsigned long previousMillis = 0;
  if ((millis() - previousMillis) >= 1000)
  { // Millise
    pumpRpm = countR * 30; // 2 pulses per turn (60/2)
    countR = 0;
    flowLPM = countF * 0.283; // LPM = 0.283 x Hz
    countF = 0;
    previousMillis += 1000;
  }
}

/*
  @brief Read coolant sensor temperature
  @param uint8_t port (0...3)
*/
void readTempCoolant(uint8_t port)
{
  float volt = abs(Myads.getAds3(port)) * 0.125; //0.125 -> ONE GAIN
  float buffercon = (vin / volt) - 1;
  float R2 = R4 * buffercon;

  switch (port)
  {
    case 0:
      filter7.AddValue(getRes10kTemperature(R2));
      tempSenCoolant[0] = filter7.GetLowPass();
      break;
    case 1:
      filter8.AddValue(getRes10kTemperature(R2));
      tempSenCoolant[1] = filter8.GetLowPass();
      break;
    case 2:
      filter9.AddValue(getRes10kTemperature(R2));
      tempSenCoolant[2] = filter9.GetLowPass();
      break;
    case 3:
      filter10.AddValue(getRes10kTemperature(R2));
      tempSenCoolant[3] = filter10.GetLowPass();
      break;
  }
}

/*
  @brief read the temperature of the peltiers through the ads1115 (1 , 2)
*/
void readTempPel()
{
  float volt = 0;
  float R2 = 0;
  float buffercon = 0;

  for (uint8_t i = 0; i < 6; i++)
  {
    if (i <= 3)
    {
      volt = abs(Myads.getAds1(i)) * 0.125; //0.125->1 gain
      buffercon = (vin / volt) - 1;
      R2 = R1 * buffercon;

      switch (i)
      {
        case 0:
          filter1.AddValue(getRes5kTemperature(R2));
          tempsPel[i] = filter1.GetLowPass();
          break;
        case 1:
          filter2.AddValue(getRes5kTemperature(R2));
          tempsPel[i] = filter2.GetLowPass();
          break;
        case 2:
          filter3.AddValue(getRes5kTemperature(R2));
          tempsPel[i] = filter3.GetLowPass();
          break;
        case 3:
          filter4.AddValue(getRes5kTemperature(R2));
          tempsPel[i] = filter4.GetLowPass();
          break;
      }
    }
    else
    {
      volt = abs(Myads.getAds2(i - 4)) * 0.125; //0.125->1 gain
      buffercon = (vin / volt) - 1;
      R2 = R3 * buffercon;

      switch (i)
      {
        case 4:
          filter5.AddValue(getRes15kTemperature(R2));
          tempsPel[i] = filter5.GetLowPass();
          break;
        case 5:
          filter6.AddValue(getRes15kTemperature(R2));
          tempsPel[i] = filter5.GetLowPass();
          break;
      }
    }
  }
}

/*
  @brief get 5k thermistor temperature
  @param uint8_t state (HIGH or LOW)
*/
float getRes5kTemperature(uint16_t res)
{
  float tx = 530.1959 * pow(res, -0.1231) - 160.7144;
  return tx;
}

/*
  @brief get 10k thermistor temperature
  @param uint8_t state (HIGH or LOW)
*/
float getRes10kTemperature(uint16_t res)
{
  float tx = 708.9896 * pow(res, -0.1471) - 157.9437;
  return tx;
}

/*
  @brief get 15k thermistor temperature
  @param uint8_t state (HIGH or LOW)
*/
float getRes15kTemperature(uint16_t res)
{
  float tx = 627.7120 * pow(res, -0.1400) - 138.3676;
  return tx;
}

/*
  @brief Set coolant pump level
  @param uint8_t state (HIGH or LOW)
*/
void writeOnPump(uint8_t state)
{
  pcf8574.digitalWrite(5, state);
  pumpState = state;
}

/*
   @brief Write pwm to peltier cells
   @param uint8_t number peltier group number (0...3)
   @param uint8_t cmode cooling or heating mode |(0 -> cooling)|(1 -> heating)|(0 -> stop)
   @param uint8_t power dutyCycle
*/
void writeTempPel(uint8_t number, uint8_t cmode, uint8_t power)
{
  switch (number)
  {
    case 0:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0)
      {
        digitalWrite(S1_C1, HIGH);
        digitalWrite(S1_C2, LOW);
      }
      else if (cmode == 1)
      {
        digitalWrite(S1_C1, LOW);
        digitalWrite(S1_C2, HIGH);
      }
      else if (cmode == 2)
      {
        digitalWrite(S1_C1, LOW);
        digitalWrite(S1_C2, LOW);
      }
      break;
    case 1:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0)
      {
        digitalWrite(S2_C1, HIGH);
        digitalWrite(S2_C2, LOW);
      }
      else if (cmode == 1)
      {
        digitalWrite(S2_C1, LOW);
        digitalWrite(S2_C2, HIGH);
      }
      else if (cmode == 2)
      {
        digitalWrite(S2_C1, LOW);
        digitalWrite(S2_C2, LOW);
      }

      break;
    case 2:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0)
      {
        digitalWrite(S3_C1, HIGH);
        digitalWrite(S3_C2, LOW);
      }
      else if (cmode == 1)
      {
        digitalWrite(S3_C1, LOW);
        digitalWrite(S3_C2, HIGH);
      }
      else if (cmode == 2)
      {
        digitalWrite(S3_C1, LOW);
        digitalWrite(S3_C2, LOW);
      }
      break;
    case 3:
      ledcWrite(PWM_CHANNEL[number], power); //ledChannel, dutyCycle
      if (cmode == 0)
      {
        digitalWrite(S4_C1, HIGH);
        digitalWrite(S4_C2, LOW);
      }
      else if (cmode == 1)
      {
        digitalWrite(S4_C1, LOW);
        digitalWrite(S4_C2, HIGH);
      }
      else if (cmode == 2)
      {
        digitalWrite(S4_C1, LOW);
        digitalWrite(S4_C2, LOW);
      }
      break;
  }
}

/*

*/
void softStartPeltier()
{
  for (uint8_t dutyCycle = 0; dutyCycle <= 127; dutyCycle++)
  {
    writeTempPel(0, 0, dutyCycle);
    writeTempPel(1, 0, dutyCycle);
    writeTempPel(2, 0, dutyCycle);
    writeTempPel(3, 0, dutyCycle);
    delay(10);
  }
}

/*
  @brief Set the chiller duty cycle via dac081c
  @param uint8_t dutyCycle (0...255)
*/
void dacSendByte(uint8_t data1)
{
  uint8_t upperdata = (data1 >> 4) & 0b00001111; //Force into normal mode, then upper 4 bits of data
  uint8_t lowerdata = (data1 << 4) & 0b11110000; //retain lower 4 bits of data, then 4 don't care

  Wire.beginTransmission(ADDR_DAC); // Transmit to addres 0xD (0001101)  device address is specified in datasheet GND
  Wire.write(upperdata);
  Wire.write(lowerdata);
  Wire.endTransmission(); // stop transmitting
}
