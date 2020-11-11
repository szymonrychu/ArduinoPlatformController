#include "Arduino.h"
#include "pins.h"

#ifndef POLOLU_VNH3SP30_H
#define POLOLU_VNH3SP30_H

#ifndef DEADZONE
#define DEADZONE 0.05f
#endif
struct HbridgePins {
    int pwmPin, dir1Pin;
};
class Hbridge {
private:
    int pwmPin = 0;
    int dir1Pin = 0;
public:

    Hbridge(){}

    Hbridge(HbridgePins pins){
        this->pwmPin = pins.pwmPin;
        this->dir1Pin = pins.dir1Pin;
    }

    Hbridge(int pwmPin, int dir1Pin){
        this->pwmPin = pwmPin;
        this->dir1Pin = dir1Pin;
    }

    void setup(){
        pinMode(pwmPin, OUTPUT);
        pinMode(dir1Pin, OUTPUT);
    }

    void drive(int power){
        if(power > (DEADZONE*(double)pow(2, PWM_RESOLUTION))){
            digitalWrite(dir1Pin, HIGH);
            analogWrite(pwmPin, power);
        }else if(power < -(DEADZONE*(double)pow(2, PWM_RESOLUTION))){
            digitalWrite(dir1Pin, LOW);
            analogWrite(pwmPin, -power);
        }else{
            digitalWrite(dir1Pin, LOW);
            analogWrite(pwmPin, 0);
        }
    }


};
#endif
