#include "Arduino.h"
#include "PID.h"

#define ADC_MAX_VALUE      1024.0

#define CURRENT_ANALOG_PIN A2
#define CURRENT_MIN       -10.0
#define CURRENT_MAX        10.0
// #define ANALOG_TO_CURRENT  1.0

#define VOLTAGE_ANALOG_PIN A3
#define ANALOG_TO_VOLTAGE  12.54/550.0

#define OUTPUT_PWM_PIN      9
#define MAX_PWM             255
#define MIN_PWM             0

#define P 1.5
#define I 0.0
#define D 0.5

#define INPUT 1.0

uint8_t dir = 1;


unsigned long prevTime = 0;
float previousError = 0;
float kI = 0;

float sumDelta = 0;

float error = 0.0;

void setup() {
  pinMode(OUTPUT_PWM_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  float timeDelta = float(currentTime - prevTime)/1000.0;
  prevTime = currentTime;


  float outputVoltage = ANALOG_TO_VOLTAGE * analogRead(VOLTAGE_ANALOG_PIN);
  float outputCurrent = -((float(analogRead(CURRENT_ANALOG_PIN)) * (CURRENT_MAX - CURRENT_MIN) / ADC_MAX_VALUE) + CURRENT_MIN);
  error = (INPUT - outputCurrent)/CURRENT_MAX;

  float kP = P*(error);

  float kD = D*(previousError - error)/timeDelta;
  previousError = error;

  kI += I*(error*timeDelta);

  float steering = kP + kI + kD;

  if(steering < 0)steering = 0.0f;
  if(steering > 1.0)steering = 1.0f;
  
  uint8_t pwm = int(255.0f * steering);

  analogWrite(OUTPUT_PWM_PIN, pwm);

  delay(100);

  

  

  sumDelta += timeDelta;
  if(sumDelta > 0.25){
    Serial.print("V:");
    Serial.print(outputVoltage);
    Serial.print(":C:");
    Serial.print(outputCurrent);
    Serial.print(":Err:");
    Serial.print(error);
    Serial.print(":P:");
    Serial.print(kP);
    Serial.print(":I:");
    Serial.print(kI);
    Serial.print(":D:");
    Serial.print(kD);
    Serial.print(":PWM:");
    Serial.print(pwm);
    Serial.print(":T:");
    Serial.println(timeDelta);
    sumDelta = 0;
  }

}
