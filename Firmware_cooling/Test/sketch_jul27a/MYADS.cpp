/*
  @file MyAds.cpp

  @Author
  Nicolas Useche
  V 1.0.0
*/
#include "MYADS.h"

ADS1115_lite ads(ADS1115_ADDRESS_ADDR_GND);  // 0x48 addr pin connected to GND
ADS1115_lite ads1(ADS1115_ADDRESS_ADDR_VDD); // 0x48 addr pin connected to GND
ADS1115_lite ads2(ADS1115_ADDRESS_ADDR_SDA); // 0x48 addr pin connected to GND
ADS1115_lite ads3(ADS1115_ADDRESS_ADDR_SCL); // 0x48 addr pin connected to GND

MYADS::MYADS()
{
  ads.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec

  ads1.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads1.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec

  ads2.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads2.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec

  ads3.setGain(ADS1115_REG_CONFIG_PGA_4_096V);     // GAIN_ONE and resolution to ± 4.096V
  ads3.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec
}

/*
  void MYADS::setAdsConfigAll(void)
  {
  }
*/

int16_t MYADS::getAds0(uint8_t port)
{
  switch (port)
  {
    case 0:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  ads.triggerConversion();    // Triggered mannually
  return ads.getConversion(); // returns int16_t value
}

int16_t MYADS::getAds1(uint8_t port)
{
  switch (port)
  {
    case 0:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  ads1.triggerConversion();    // Triggered mannually
  return ads1.getConversion(); // returns int16_t value
}

int16_t MYADS::getAds2(uint8_t port)
{
  switch (port)
  {
    case 0:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  ads2.triggerConversion(); // Triggered mannually
  return ads2.getConversion();
}

int16_t MYADS::getAds3(uint8_t port)
{
  switch (port)
  {
    case 0:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 1:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 2:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2); // Single mode input on AIN0 (AIN0 - GND)
      break;
    case 3:
      ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3); // Single mode input on AIN0 (AIN0 - GND)
      break;
  }
  ads3.triggerConversion();    // Triggered mannually
  return ads3.getConversion(); // returns int16_t value
}
