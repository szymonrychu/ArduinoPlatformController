#include "Arduino.h"

#define CHARGING_ENABLE 2
#define POWER_ENABLE 3
#define SYSTEM_ENABLE 15

#define CHARGING_ADC A3
#define BATTERY_ADC A2

#define CHG_FULL_INDICATOR 6

#define BATTERY_MAX_VOLTAGE 4.2*3
#define BATTERY_MIN_VOLTAGE 3.1*3
#define BATTERY_MID_VOLTAGE (BATTERY_MAX_VOLTAGE+BATTERY_MIN_VOLTAGE)/2

#define CHARGING_MIN_VOLTAGE 15

#define MEAN_POINTS_NUM 100
float batteryVoltages[MEAN_POINTS_NUM] = {};
float chargingVoltages[MEAN_POINTS_NUM] = {};
int voltagesPointer = 0;
bool histerezis = true;

void setup() {
  pinMode(CHARGING_ENABLE, OUTPUT);
  digitalWrite(CHARGING_ENABLE, HIGH);
  pinMode(POWER_ENABLE, OUTPUT);
  digitalWrite(POWER_ENABLE, HIGH);
  pinMode(SYSTEM_ENABLE, OUTPUT);
  digitalWrite(SYSTEM_ENABLE, HIGH);
  pinMode(CHG_FULL_INDICATOR, INPUT_PULLDOWN);

  Serial.begin(115200);
  for(int c=0; c<MEAN_POINTS_NUM;c++){
    batteryVoltages[c] = 0.0;
    chargingVoltages[c] = 0.0;
  }
}
void loop() {
  bool systemON = false;
  bool powerON = false;
  bool chargingON = false;
  bool batteryFull = digitalRead(CHG_FULL_INDICATOR);
  float batteryRawVoltage = 12.25*float(analogRead(BATTERY_ADC))/488.0;
  float chargingRawVoltage = 20.5*float(analogRead(CHARGING_ADC))/807.0;
  batteryVoltages[voltagesPointer] = batteryRawVoltage;
  chargingVoltages[voltagesPointer] = chargingRawVoltage;
  voltagesPointer = (voltagesPointer + 1) % MEAN_POINTS_NUM;
  float batteryVoltageSum = 0.0f;
  float chargingVoltageSum = 0.0f;
  for(int c=0; c<MEAN_POINTS_NUM;c++){
    batteryVoltageSum += batteryVoltages[c];
    chargingVoltageSum += chargingVoltages[c];
  }
  float batteryVoltage = batteryVoltageSum / float(MEAN_POINTS_NUM);
  float chargingVoltage = chargingVoltageSum / float(MEAN_POINTS_NUM);

  if((!batteryFull || batteryVoltage<BATTERY_MID_VOLTAGE) && chargingVoltage>CHARGING_MIN_VOLTAGE ){
    chargingON = true;
  }
  if(batteryVoltage < BATTERY_MIN_VOLTAGE){
    histerezis = false;
  }
  if(batteryVoltage > BATTERY_MID_VOLTAGE){
    histerezis = true;
  }
  if(histerezis){
    systemON = true;
    powerON = true;
  }
  digitalWrite(CHARGING_ENABLE, chargingON ? LOW : HIGH);
  digitalWrite(POWER_ENABLE, powerON ? LOW : HIGH);
  digitalWrite(SYSTEM_ENABLE, systemON ? HIGH : LOW);

  if(Serial){
    Serial.print(batteryFull ? "T" : "F");
    Serial.print(",");
    Serial.print(systemON ? "T" : "F");
    Serial.print(",");
    Serial.print(powerON ? "T" : "F");
    Serial.print(",");
    Serial.print(chargingON ? "T" : "F");
    Serial.print(":");
    Serial.print(batteryVoltage, 2);
    Serial.print(",");
    Serial.print(chargingVoltage, 2);
    Serial.println("");
  }
}