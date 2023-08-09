#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

class Battery {
private:
    uint8_t batteryPin = 0;
public:

    void setup(uint8_t batteryPin){
        this->batteryPin = batteryPin;
        pinMode(this->batteryPin, INPUT);
    }

    double readVoltage(){
        return 10.82*((float)analogRead(this->batteryPin))/498.0;
    }
};

#endif