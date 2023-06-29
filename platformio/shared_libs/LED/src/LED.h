#ifndef LED_H
#define LED_H

#include "Arduino.h"

class Led{
private:
  bool ledState = false;
  uint16_t ledPin = 0;

  void refresh(){
    int state = LOW;
    if(this->ledState) state = HIGH;
    digitalWrite(this->ledPin, state);
  }

public:
  Led(int pin, int initialHigh=false){
    this->ledPin = pin;
    this->ledState = initialHigh;
    pinMode(this->ledPin, OUTPUT);
    refresh();
  }

  void toggle(){
    this->ledState = ! this->ledState;
    refresh();
  }

  bool state(){
    return this->ledState;
  }

  void setState(bool high){
    this->ledState = high;
    refresh();
  }
};

#endif