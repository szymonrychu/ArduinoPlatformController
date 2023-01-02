#include "Arduino.h"
#include "Bounce2.h"

#ifndef JOY_BUTTON_h
#define JOY_BUTTON_h
class JoyButton {
private:
    Bounce debouncer;
    uint8_t pinNumber = 0;
    bool lastState = LOW;
public:
    JoyButton(int pin){
        this->debouncer = Bounce();
        this->pinNumber = pin;
        pinMode(this->pinNumber, INPUT_PULLUP);
        this->debouncer.attach(this->pinNumber);
        this->debouncer.interval(1); // interval in ms
    }

    bool handle(){
        if(this->debouncer.update()){
            lastState = ! this->debouncer.read();
        }
        return lastState;
    }

};
#endif