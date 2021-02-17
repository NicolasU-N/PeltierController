#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
#define relay1 3
#define relay2 4
int lowtemp=6;
int hightemp=8;
 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 
void setup(void)
{
  Serial.begin(9600);
  sensors.begin();
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
}
 
 
void loop(void)
{
  sensors.requestTemperatures(); 
  Serial.print(sensors.getTempCByIndex(0));
  if (sensors.getTempCByIndex(0) < lowtemp){
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);}
  if (sensors.getTempCByIndex(0) > hightemp){
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);}
  }
    
