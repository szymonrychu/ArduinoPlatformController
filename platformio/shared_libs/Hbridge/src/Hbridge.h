#include "Arduino.h"

#ifndef POLOLU_VNH3SP30_H
#define POLOLU_VNH3SP30_H

#ifndef MOTOR_DEADZONE
#define MOTOR_DEADZONE 0.05f
#endif

#define TYPE_2_PIN 0
#define TYPE_3_PIN 1

struct HbridgePins {
    int pwmPin = 0;
    int dir1Pin = 0;
    int dir2Pin = 0;
};

class Hbridge {
private:
    int pwmPin = 0;
    int dir1Pin = 0;
    int dir2Pin = 0;
    int hBridgeType = TYPE_2_PIN;
public:

    Hbridge(){}

    Hbridge(int pwmPin, int dir1Pin, int dir2Pin=0){
        this->pwmPin = pwmPin;
        this->dir1Pin = dir1Pin;
        if(dir2Pin != 0){
            this->dir2Pin = dir2Pin;
            pinMode(dir2Pin, OUTPUT);
            this->hBridgeType = TYPE_3_PIN;
        }
        pinMode(pwmPin, OUTPUT);
        pinMode(dir1Pin, OUTPUT);
    }

    Hbridge(HbridgePins pins){
        Hbridge(pins.pwmPin, pins.dir1Pin, pins.dir2Pin);
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
