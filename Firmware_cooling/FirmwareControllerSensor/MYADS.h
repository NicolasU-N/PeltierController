/*
  @file MyAds.h

  @Author
  Nicolas Useche
  V 1.0.0
*/

#ifndef MYADS_h
#define MYADS_h

#include "Arduino.h"
#include <Wire.h>
#include <ADS1115_lite.h>

class MYADS
{
    //private:

  public:
    MYADS();
    //void setAdsConfigAll();

    int16_t getAds0(uint8_t port);
    int16_t getAds1(uint8_t port);
    int16_t getAds2(uint8_t port);
    int16_t getAds3(uint8_t port);
};
#endif
