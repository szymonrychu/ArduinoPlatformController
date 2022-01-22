#include "Arduino.h"

#ifndef JOY_ANALOG_h
#define JOY_ANALOG_h

#define POTENTIOMETER_MIN 0
#define POTENTIOMETER_MAX 1023
#define ADC_MIN 0
#define ADC_MAX 1023

class JoyAnalog {
private:
    uint8_t pinNumber = 0;

    int mapPotentiometerValue(int rawValue) {
        int translatedValue = map(rawValue, POTENTIOMETER_MIN, POTENTIOMETER_MAX, 0, 1023);
        return max(ADC_MIN, min(ADC_MAX, translatedValue));
    }
public:
    JoyAnalog(int pin){
        this->pinNumber = pin;
    }

    int handle(){
        return mapPotentiometerValue(analogRead(this->pinNumber));
    }
};

#endif