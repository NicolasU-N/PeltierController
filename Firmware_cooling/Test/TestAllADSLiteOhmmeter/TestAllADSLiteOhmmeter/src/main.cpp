// resistencia term 11.5kohm pequeños 5k
// resistencia term 33.0kohm  grandes 15k

#include <Arduino.h>
#include "MYADS.h"
#include "SingleEMAFilterLib.h"

SingleEMAFilter<float> filter1(0.4); // 0.2
SingleEMAFilter<float> filter2(0.4); // 0.2
SingleEMAFilter<float> filter3(0.4); // 0.2
SingleEMAFilter<float> filter4(0.4); // 0.2
SingleEMAFilter<float> filter5(0.4); // 0.2
SingleEMAFilter<float> filter6(0.4); // 0.2

MYADS Myads;

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ PELTIERS
float vin = 3260;
float tempsPel[6];
float R1 = 325;
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

void readTempPel();
float getRes5kTemperature(float res);
float getRes15kTemperature(float res);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  //Serial.println(getRes5kTemperature(11500));
  readTempPel();
  delay(500);
}

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
      R2 = R1 * buffercon;

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
    Serial.print("|");
    Serial.print(tempsPel[i]);
    Serial.print("|");
  }
  Serial.println("");
}

float getRes5kTemperature(float res)
{
  float tx = 530.1959 * pow(res, -0.1231) - 160.7144;
  return tx;
}

float getRes15kTemperature(float res)
{
  float tx = 627.7120 * pow(res, -0.1400) - 138.3676;
  return tx;
}

float getRes10kTemperature(uint16_t res)
{
  float tx = 708.9896 * pow(res, -0.1471) - 157.9437;
  return tx;
}