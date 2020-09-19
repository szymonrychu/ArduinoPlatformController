#ifndef PINS_h
#define PINS_h

#include "Arduino.h"

#define LOOP_T 1
#define TICKS2SEC 100
#define DIR_DIST 1
#define LOG_ACCURACY 2

#define DIST_PWM  22 //ena
#define DIST_P1   21 //in2

#define ANGL_PWM  22 //enb
#define ANGL_P1   23 //in4

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
}

#define DEADZONE 0.0f
#define PID_DELTA_TOLERANCE 0.0f

#define PID_DGAIN 1.0f
#define PID_VGAIN 1.0f

#define DIST_D_P PID_DGAIN*0.000001 //25f
#define DIST_D_I PID_DGAIN*0.0 //0001f
#define DIST_D_D PID_DGAIN*0.0 //002f

#define DIST_V_P PID_VGAIN*0.00001 //15f
#define DIST_V_I PID_VGAIN*0.0f
#define DIST_V_D PID_VGAIN*0.0 //04f
#define DIST_IDLE_VELOCITY 5.0f

#define DIST_ENC2DIST 56.8f/3198.48530706f // encoder to cm

#define ANGL_D_P PID_DGAIN*0.035f
#define ANGL_D_I PID_DGAIN*0.0002f
#define ANGL_D_D PID_DGAIN*0.004f

#define ANGL_V_P PID_VGAIN*0.015f
#define ANGL_V_I PID_VGAIN*0.0f
#define ANGL_V_D PID_VGAIN*0.004f
#define ANGL_IDLE_VELOCITY 5.0f

#define ANGL_ENC2DIST (8.9f/10.0f)*(90.0f/55.0f)*56.8f/3198.48530706f // encoder to angle

#define MAX_PID     1.0f
#define MIN_DPID   -1.0f
#define MIN_VPID    0.0f

#endif
