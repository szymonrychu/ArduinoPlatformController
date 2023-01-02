#include "Arduino.h"

#define ANALOG_PIN_VOLTAGE A1
#define ANALOG_READ_TO_VOLTAGE 12.57/614.0

#define ANALOG_PIN_CURRENT A3
#define ANALOG_READ_TO_CURRENT 2.55/524.0

#define MEM_LEN 256
char buffer[MEM_LEN];

float maxCurrent = 0.0;

void setup() {                
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {
  float voltage = ANALOG_READ_TO_VOLTAGE * (float)analogRead(ANALOG_PIN_VOLTAGE);
  float current = ((ANALOG_READ_TO_CURRENT * (float)analogRead(ANALOG_PIN_CURRENT))-2.5)/0.45;

  if(current > maxCurrent) maxCurrent = current;

  // Logger::info("", false);
  Serial.print(voltage, 4);
  Serial.print(":");
  Serial.print(current, 4);
  Serial.print(":");
  Serial.println(maxCurrent, 4);
  delay(1000);
}


// #define ANALOG_READ_RESOLUTION           12
// #define ANALOG_WRITE_RESOLUTION          12
// #define ANALOG_READ_MAX_VALUE            2**ANALOG_READ_RESOLUTION
// #define ANALOG_WRITE_MAX_VALUE           2**ANALOG_WRITE_RESOLUTION
// #define ANALOG_WRITE_FREQUENCY           2500

// #define POWER_ON_OUT_PIN            22
// #define OUTPUT_ENABLE_PIN           5
// #define batteryVoltage_ANALOG_PIN  A5
// #define POWER_OUTPUT_ANALOG_PIN     A6

// #define MIN_CELL_VOLTAGE_CUTOFF     3.0
// #define MIN_CELL_VOLTAGE_RE_ENABLE  3.1


// #define MOVING_AVERAGE_READINGS_N 100

// class MovingAverage{
//   private:
//     float readings[MOVING_AVERAGE_READINGS_N];
//     uint16_t currentReadingsCount = MOVING_AVERAGE_READINGS_N;
//     uint16_t currentPointer = 0;
//   public:
//     float lastReading = 0;

//     MovingAverage(){
//       this->currentReadingsCount = MOVING_AVERAGE_READINGS_N;
//       for(int c=0; c<MOVING_AVERAGE_READINGS_N; c++){
//         readings[c]=0.0;
//       }
//     }

//     float update(float reading){
//       lastReading = reading;
//       readings[currentPointer] = reading;
//       currentPointer = (currentPointer + 1)%MOVING_AVERAGE_READINGS_N;
//       if(currentReadingsCount > 1){
//         currentReadingsCount--;
//         return 0.0;
//       }else{
//         float sum = 0.0;
//         for(int c=0; c<MOVING_AVERAGE_READINGS_N; c++){
//           sum += readings[c];
//         }
//         return sum/MOVING_AVERAGE_READINGS_N;
//       }
//     }


// };

// uint64_t msgCounter = 0;
// bool outputEnabled = false;
// bool cutOffVoltageHit = false;
// bool cutOffCurrentHit = false;
// uint64_t lastTimestamp = 0;
// MovingAverage voltage;
// MovingAverage current;


// void setup() {
//   pinMode(OUTPUT_ENABLE_PIN, OUTPUT);
//   digitalWrite(OUTPUT_ENABLE_PIN, HIGH);

//   pinMode(POWER_ON_OUT_PIN, OUTPUT);
//   digitalWrite(POWER_ON_OUT_PIN, HIGH);
  
//   analogWriteResolution(ANALOG_WRITE_RESOLUTION);
//   analogReadResolution(ANALOG_READ_RESOLUTION);
// }



// void loop(){

//   digitalWrite(POWER_ON_OUT_PIN, HIGH);
//   delay(1);
//   float batteryVoltage = voltage.update(11.43 * analogRead(batteryVoltage_ANALOG_PIN) / 650.0);
//   digitalWrite(POWER_ON_OUT_PIN, LOW);


//   float outputCurrent = current.update(5.1 * ((11.43 * analogRead(POWER_OUTPUT_ANALOG_PIN) / 650.0)-3.31) );
//   if(current.lastReading > 15.0){
//     cutOffCurrentHit = true;
//   }

//   if(cutOffCurrentHit){
//     outputEnabled = false;
//   }else{
//     if(cutOffVoltageHit){
//       if(batteryVoltage > 3*MIN_CELL_VOLTAGE_RE_ENABLE){
//         outputEnabled = true;
//         cutOffVoltageHit = false;
//       }
//     }else{
//       if(batteryVoltage > 3*MIN_CELL_VOLTAGE_CUTOFF){
//         outputEnabled = true;
//       }else{
//         outputEnabled = false;
//         cutOffVoltageHit = true;
//       }
//     }
//   }
  

//   digitalWrite(OUTPUT_ENABLE_PIN, outputEnabled ? HIGH : LOW);

//   if(batteryVoltage > 0 || outputCurrent > 0){
//     lastTimestamp++;
//     lastTimestamp = (lastTimestamp+1)%10;
//     if(lastTimestamp == 0){
//       Serial.printf("%lu:%.2f:%.2f:%s\n", msgCounter++, batteryVoltage, outputCurrent, outputEnabled ? "T" : "F");
//     }
//   }
//   delay(100);
// }
