#include "Arduino.h"

#ifndef POLOLU_VNH3SP30_H
#define POLOLU_VNH3SP30_H

#ifndef MOTOR_DEADZONE
#define MOTOR_DEADZONE 0.05f
#endif

#ifndef PWM_RESOLUTION
#define PWM_RESOLUTION 12
#endif

#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 30000
#endif

#define TYPE_2_PIN 0
#define TYPE_3_PIN 1

class Hbridge {
private:
    int pwmPin = 0;
    int dir1Pin = 0;
    int dir2Pin = 0;
    int hBridgeType = TYPE_2_PIN;
public:

    Hbridge(){}

    Hbridge(int pwmPin, int dir1Pin, int dir2Pin=0, int pwmFrequency=PWM_FREQUENCY){
        this->pwmPin = pwmPin;
        this->dir1Pin = dir1Pin;
        pinMode(pwmPin, OUTPUT);
        analogWriteFrequency(pwmPin, pwmFrequency);
        pinMode(dir1Pin, OUTPUT);
        if(dir2Pin != 0){
            this->hBridgeType = TYPE_3_PIN;
            this->dir2Pin = dir2Pin;
            pinMode(dir2Pin, OUTPUT);
        }
    }

    bool drive(int power){
        if(power > (MOTOR_DEADZONE*(double)pow(2, PWM_RESOLUTION))){
            digitalWrite(dir1Pin, HIGH);
            if(this->hBridgeType == TYPE_3_PIN){
                digitalWrite(dir2Pin, LOW);
            }
            analogWrite(pwmPin, power);
            return true;
        }else if(power < -(MOTOR_DEADZONE*(double)pow(2, PWM_RESOLUTION))){
            digitalWrite(dir1Pin, LOW);
            if(this->hBridgeType == TYPE_3_PIN){
                digitalWrite(dir2Pin, HIGH);
            }
            analogWrite(pwmPin, -power);
            return true;
        }else{
            digitalWrite(dir1Pin, LOW);
            if(this->hBridgeType == TYPE_3_PIN){
                digitalWrite(dir2Pin, LOW);
            }
            analogWrite(pwmPin, 0);
            return false;
        }
    }


};
#endif
