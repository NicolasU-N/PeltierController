#include<ADS1115_lite.h>
ADS1115_lite ads(ADS1115_DEFAULT_ADDRESS);  // 0x48 addr pin connected to GND

void setup() {
  Serial.begin(115200);
  delay(10);
  ads_config(); // call to ads_config function
}

void loop() {
  
  Serial.print((int)ads_read());
  Serial.print("     ----    ");
  Serial.print((int)ads_read1());
  Serial.print("     ----    ");
  Serial.print((int)ads_read2());
  Serial.print("     ----    ");
  Serial.println((int)ads_read3());
  
  delay(500);
}

void ads_config() {
  ads.setGain(ADS1115_REG_CONFIG_PGA_4_096V); // GAIN_ONE and resolution to ± 4.096V
  ads.setSampleRate(ADS1115_REG_CONFIG_DR_860SPS); // Set to the fastest MODE 860Samples per sec
}

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
