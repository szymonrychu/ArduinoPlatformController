#ifndef PINS_h
#define PINS_h

#include "Arduino.h"

#define LOOP_T 10
#define TICKS2SEC 100
#define DIR_DIST 1
#define DEADZONE 0.0f

#define DIST_ENC1 14
#define DIST_ENC2 15
#define DIST_PWM  10 //ena
#define DIST_P1   12 //in2

#define ANGL_ENC1 16
#define ANGL_ENC2 17
#define ANGL_PWM  9 //enb
#define ANGL_P1   11 //in4

#define ANGL_OPTO 13

#define PWM_RESOLUTION 11
#define PWM_FREQUENCY 20000

void pinsSetup(void (*distanceInterruptHandler)(), void (*angleInterruptHandler)()){
    int pwmPinArray[] = {
        DIST_PWM, ANGL_PWM
    };
    int outputPinArray[] = {
        DIST_P1, ANGL_P1
    };
    int inputPins[] = {
        DIST_ENC1, DIST_ENC2, ANGL_ENC1, ANGL_ENC2
    };
    analogWriteResolution(PWM_RESOLUTION);
    for(int c=0; c<2; c++){
        pinMode(pwmPinArray[c], OUTPUT);
        analogWriteFrequency(pwmPinArray[c], PWM_FREQUENCY);
        analogWrite(pwmPinArray[c], 0);
    }
    for(int c=0; c<2; c++){
        pinMode(outputPinArray[c], OUTPUT);
        digitalWrite(outputPinArray[c], LOW);
    }
    for(int c=0; c<4; c++){
        pinMode(inputPins[c], INPUT);
    }
    
    attachInterrupt(digitalPinToInterrupt(DIST_ENC1), distanceInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DIST_ENC2), distanceInterruptHandler, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(ANGL_ENC1), angleInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ANGL_ENC2), angleInterruptHandler, CHANGE);
}

void pinOptoSetup(void (*optoInterrupt)()){
    pinMode(ANGL_OPTO, INPUT);
    #ifdef USB_SERIAL
    attachInterrupt(digitalPinToInterrupt(ANGL_OPTO), optoInterrupt, LOW);
    #else
    attachInterrupt(digitalPinToInterrupt(ANGL_OPTO), optoInterrupt, HIGH);
    #endif
}

#define PID_DGAIN 0.01f
#define PID_VGAIN 0.2f

#define DIST_D_P PID_DGAIN*7.5f
#define DIST_D_I PID_DGAIN*0.0f
#define DIST_D_D PID_DGAIN*1.8f

#define DIST_V_P PID_VGAIN*7.5f
#define DIST_V_I PID_VGAIN*0.0f
#define DIST_V_D PID_VGAIN*1.8f

#define DIST_ENC2DIST 56.8f/3198.48530706f // encoder to cm

#define ANGL_D_P PID_DGAIN*7.5f
#define ANGL_D_I PID_DGAIN*0.0f
#define ANGL_D_D PID_DGAIN*1.8f

#define ANGL_V_P PID_VGAIN*7.5f
#define ANGL_V_I PID_VGAIN*0.0f
#define ANGL_V_D PID_VGAIN*1.8f

#define ANGL_ENC2DIST 56.8f/3198.48530706f // encoder to angle

#define MAX_PID     1.0f
#define MIN_DPID   -1.0f
#define MIN_VPID    0.0f

#endif
