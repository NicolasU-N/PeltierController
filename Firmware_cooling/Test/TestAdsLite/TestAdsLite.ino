#include<ADS1115_lite.h>

ADS1115_lite ads(ADS1115_ADDRESS_ADDR_GND);  // 0x48 addr pin connected to GND
ADS1115_lite ads1(ADS1115_ADDRESS_ADDR_VDD);  // 0x48 addr pin connected to GND
ADS1115_lite ads2(ADS1115_ADDRESS_ADDR_SDA);  // 0x48 addr pin connected to GND
ADS1115_lite ads3(ADS1115_ADDRESS_ADDR_SCL);  // 0x48 addr pin connected to GND

int16_t tempsPel[10];

float vin = 3260;
float volt = 0;
float R1 = 325;
float R2 = 0;
float buffercon = 0;

void setup() {
  Serial.begin(115200);
  delay(10);
  ads_config_all(); // call to ads_config function
}

void loop() {

  //for (uint8_t i = 0; i < 10; i++) {
  //  tempsPel[i] = readTempPel(i);
  //}

  volt = (float)ads2_read() * 0.125; //voltaje 0.125 -> 1 gain // 0.1875 -> 2/3 gain
  buffercon = (vin / volt) - 1;
  R2 = R1 * buffercon;
  Serial.println(R2);

  delay(500);
}

void ads_config_all() {
  ads.setGain(ADS1115_REG_CONFIG_PGA_4_096V); // GAIN_ONE and resolution to ± 4.096V
  ads.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec

  ads1.setGain(ADS1115_REG_CONFIG_PGA_4_096V); // GAIN_ONE and resolution to ± 4.096V
  ads1.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec

  ads2.setGain(ADS1115_REG_CONFIG_PGA_4_096V); // GAIN_ONE and resolution to ± 4.096V
  ads2.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec

  ads3.setGain(ADS1115_REG_CONFIG_PGA_4_096V); // GAIN_ONE and resolution to ± 4.096V
  ads3.setSampleRate(ADS1115_REG_CONFIG_DR_64SPS); // Set to the fastest MODE 860Samples per sec
}

///////////////////////////////////////////////////////////////////////////////////////////                   ADS 0

int16_t ads_read() {
  ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // Single mode input on AIN0 (AIN0 - GND)
  ads.triggerConversion();  // Triggered mannually
  return ads.getConversion();  // returns int16_t value
}

int16_t ads_read1() {
  ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);  // Single mode input on AIN0 (AIN0 - GND)
  ads.triggerConversion();  // Triggered mannually
  return ads.getConversion();  // returns int16_t value
}

int16_t ads_read2() {
  ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);  // Single mode input on AIN0 (AIN0 - GND)
  ads.triggerConversion();  // Triggered mannually
  return ads.getConversion();  // returns int16_t value
}

int16_t ads_read3() {
  ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);  // Single mode input on AIN0 (AIN0 - GND)
  ads.triggerConversion();  // Triggered mannually
  return ads.getConversion();  // returns int16_t value
}


///////////////////////////////////////////////////////////////////////////////////////////                   ADS 1

int16_t ads1_read() {
  ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // Single mode input on AIN0 (AIN0 - GND)
  ads1.triggerConversion();  // Triggered mannually
  return ads1.getConversion();  // returns int16_t value
}

int16_t ads1_read1() {
  ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);  // Single mode input on AIN0 (AIN0 - GND)
  ads1.triggerConversion();  // Triggered mannually
  return ads1.getConversion();  // returns int16_t value
}

int16_t ads1_read2() {
  ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);  // Single mode input on AIN0 (AIN0 - GND)
  ads1.triggerConversion();  // Triggered mannually
  return ads1.getConversion();  // returns int16_t value
}

int16_t ads1_read3() {
  ads1.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);  // Single mode input on AIN0 (AIN0 - GND)
  ads1.triggerConversion();  // Triggered mannually
  return ads1.getConversion();  // returns int16_t value
}

///////////////////////////////////////////////////////////////////////////////////////////                  ADS 2

int16_t ads2_read() {
  ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // Single mode input on AIN0 (AIN0 - GND)
  ads2.triggerConversion();  // Triggered mannually
  return ads2.getConversion();  // returns int16_t value
}

int16_t ads2_read1() {
  ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);  // Single mode input on AIN0 (AIN0 - GND)
  ads2.triggerConversion();  // Triggered mannually
  return ads2.getConversion();  // returns int16_t value
}

int16_t ads2_read2() {
  ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);  // Single mode input on AIN0 (AIN0 - GND)
  ads2.triggerConversion();  // Triggered mannually
  return ads2.getConversion();  // returns int16_t value
}

int16_t ads2_read3() {
  ads2.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);  // Single mode input on AIN0 (AIN0 - GND)
  ads2.triggerConversion();  // Triggered mannually
  return ads2.getConversion();  // returns int16_t value
}


///////////////////////////////////////////////////////////////////////////////////////////                  ADS 3

int16_t ads3_read() {
  ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // Single mode input on AIN0 (AIN0 - GND)
  ads3.triggerConversion();  // Triggered mannually
  return ads3.getConversion();  // returns int16_t value
}

int16_t ads3_read1() {
  ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);  // Single mode input on AIN0 (AIN0 - GND)
  ads3.triggerConversion();  // Triggered mannually
  return ads3.getConversion();  // returns int16_t value
}

int16_t ads3_read2() {
  ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);  // Single mode input on AIN0 (AIN0 - GND)
  ads3.triggerConversion();  // Triggered mannually
  return ads3.getConversion();  // returns int16_t value
}

int16_t ads3_read3() {
  ads3.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);  // Single mode input on AIN0 (AIN0 - GND)
  ads3.triggerConversion();  // Triggered mannually
  return ads3.getConversion();  // returns int16_t value
}



/*
  0...9
*/
int16_t readTempPel(uint8_t number) {
  int16_t vout = 0;

  if (number >= 0 and number <= 3) { //T1-T4

    if (number == 0) {
      vout = ads_read();
    }

    if (number == 1) {
      vout = ads_read1();
    }

    if (number == 2) {
      vout = ads_read2();
    }

    if (number == 3) {
      vout = ads_read3();
    }

    Serial.print("ADS1__");
    Serial.print(number);
    Serial.print("__");
    Serial.print((int)vout);
    Serial.print("__");
    Serial.println("");

  }

  else if (number > 3  and number <= 7) { //T5-T8

    if (number == 3) {
      vout = ads1_read();
    }

    if (number == 4) {
      vout = ads1_read1();
    }

    if (number == 5) {
      vout = ads1_read2();
    }

    if (number == 6) {
      vout = ads1_read3();
    }

    //vout = ads1.readADC_SingleEnded(number - 4);// * (3.3 / 32767); //0...32767 resolution

    Serial.print("ADS2__");
    Serial.print(number);
    Serial.print("__");
    Serial.print((int)vout);
    Serial.print("__");
    Serial.println("");
  } else if (number > 7) { //T9-T10

    if (number == 7) {
      vout = ads2_read();
    }

    if (number == 8) {
      vout = ads2_read1();
    }

    if (number == 9) {
      vout = ads2_read2();
    }

    if (number == 10) {
      vout = ads2_read3();
    }

    //vout = ads2.readADC_SingleEnded(number - 8);// * (3.3 / 32767); //0...32767 resolution
    Serial.print("ADS3__");
    Serial.print(number);
    Serial.print("__");
    Serial.print((int)vout);
    Serial.print("__");
    Serial.println("");
  }

  //float Temp_C = Temp_K - 273.15; //converting into Celsius
  return vout;//Temp_C;

  Serial.println("");
}
