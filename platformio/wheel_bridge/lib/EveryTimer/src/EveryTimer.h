#ifndef EVERY_TIMER_H
#define EVERY_TIMER_H

#include "Arduino.h"

class EveryTimer{
private:
    unsigned long lastTime = 0;
    float everyS = 0;
    void (*handler)(void);
public:
    EveryTimer(float everyS, void (*handler)()){
        this->everyS = everyS;
        this->handler = handler;
    }

    void compute(){
        unsigned long currentTime = micros();
        if(currentTime - lastTime > (unsigned long)(everyS*1000000.0)){
            this->handler();
            lastTime = currentTime;
        }
    }


};

#endif