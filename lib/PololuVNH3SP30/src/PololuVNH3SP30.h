#include "Arduino.h"

#ifndef POLOLU_VNH3SP30_H
#define POLOLU_VNH3SP30_H
struct PololuVNH3SP30Pins {
    int pwmPin, dir1Pin;
};
class PololuVNH3SP30 {
private:
    int pwmPin = 0;
    int dir1Pin = 0;
public:

    PololuVNH3SP30(){}

    PololuVNH3SP30(PololuVNH3SP30Pins pins){
        this->pwmPin = pins.pwmPin;
        this->dir1Pin = pins.dir1Pin;
    }

    PololuVNH3SP30(int pwmPin, int dir1Pin, int dir2Pin, int csPin){
        this->pwmPin = pwmPin;
        this->dir1Pin = dir1Pin;
    }

    void setup(){
        pinMode(pwmPin, OUTPUT);
        pinMode(dir1Pin, OUTPUT);
    }

    void drive(int power){
        if(power > 0.05f*pow(2, PWM_RESOLUTION)){
            digitalWrite(dir1Pin, HIGH);
            analogWrite(pwmPin, power);
        }else if(power < -0.05f*pow(2, PWM_RESOLUTION)){
            digitalWrite(dir1Pin, LOW);
            analogWrite(pwmPin, -power);
        }else{
            digitalWrite(dir1Pin, LOW);
            analogWrite(pwmPin, 0);
        }
    }


};
#endif
