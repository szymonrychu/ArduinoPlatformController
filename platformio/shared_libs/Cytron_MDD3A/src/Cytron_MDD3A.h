#ifndef CYTRON_MDD3A_H
#define CYTRON_MDD3A_H
#include "Arduino.h"

#ifndef PWM_RESOLUTION
#define PWM_RESOLUTION 12
#endif

#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 22000
#endif

class MDD3AHbridge{
private:
  int aPin = 255;
  int bPin = 255;
public:

  MDD3AHbridge(){};

  MDD3AHbridge(int aPin, int bPin){
    this->aPin = aPin;
    this->bPin = bPin;
  };

  void setup(){
    pinMode(this->aPin, OUTPUT);
    pinMode(this->bPin, OUTPUT);
    
    analogWriteResolution(PWM_RESOLUTION);
    analogWriteFrequency(this->aPin, PWM_FREQUENCY);
    analogWriteFrequency(this->bPin, PWM_FREQUENCY);

    analogWrite(this->aPin, 0);
    analogWrite(this->bPin, 0);
  }

  void drive(double power){
    int64_t totalPower = (int)(power * (double)pow(2, PWM_RESOLUTION));
    if(totalPower > 0){
      analogWrite(this->aPin, totalPower);
      analogWrite(this->bPin, 0);
    }else if(totalPower < 0){
      analogWrite(this->aPin, 0);
      analogWrite(this->bPin, -totalPower);
    }else{
      analogWrite(this->aPin, 0);
      analogWrite(this->bPin, 0);
    }
  }
};

#endif