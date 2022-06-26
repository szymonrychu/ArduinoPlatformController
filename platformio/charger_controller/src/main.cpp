#include "Arduino.h"
#include "PID.h"


#define ANALOG_READ_RESOLUTION           12
#define ANALOG_READ_MAX_VALUE            4096.0
#define ANALOG_READ_MIN_VALUE            0

#define ANALOG_WRITE_RESOLUTION           12
#define ANALOG_WRITE_MAX_VALUE            4096.0
#define ANALOG_WRITE_MIN_VALUE            0
#define ANALOG_WRITE_FREQUENCY            2500

#define ADC_MAX_VALUE      1024.0

#define CURRENT_ANALOG_PIN A7
#define CURRENT_MIN       -10.0
#define CURRENT_MAX        10.0
#define ANALOG_TO_CURRENT  1.0

#define VOLTAGE_ANALOG_PIN A8
#define ANALOG_TO_VOLTAGE   19.13/1840

#define OUTPUT_PWM_PIN      10


#define P 0.08
#define I 0.0
#define D 0.0

#define INPUT 12.0

uint8_t dir = 1;


unsigned long prevTime = 0;
float previousError = 0;
float kI = 0;

float sumDelta = 0;
float error = 0.0;

float steering = 0.0f;
float delta = 0.1;

void setup() {
  pinMode(OUTPUT_PWM_PIN, OUTPUT);
  analogWriteFrequency(OUTPUT_PWM_PIN, ANALOG_WRITE_FREQUENCY);
  
  analogWriteResolution(ANALOG_WRITE_RESOLUTION);
  analogReadResolution(ANALOG_READ_RESOLUTION);

  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  float timeDelta = float(currentTime - prevTime)/1000.0;
  prevTime = currentTime;


  float outputVoltage = ANALOG_TO_VOLTAGE * analogRead(VOLTAGE_ANALOG_PIN);
  float outputCurrent = -((float(analogRead(CURRENT_ANALOG_PIN)) * (CURRENT_MAX - CURRENT_MIN) / ADC_MAX_VALUE) + CURRENT_MIN);
  error = INPUT - outputVoltage;

  float kP = P*(error);

  float kD = D*(previousError - error)/timeDelta;
  previousError = error;

  kI += I*(error*timeDelta);

  float steering = kP + kI + kD;


  // steering = steering + delta;
  if(steering < 0){
    steering = 0.0f;
    // delta = 0.01;
  }
  if(steering > 1.0){
    steering = 0.0f;
    // delta = -0.01;
  }
  
  unsigned int pwm = int(ANALOG_WRITE_MAX_VALUE * steering);

  analogWrite(OUTPUT_PWM_PIN, pwm);

  delay(25);

  

  

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
