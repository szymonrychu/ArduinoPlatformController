
#include "pins.h"
#ifndef PID_H
#define PID_H

#ifndef PID_MAX_INTEGRAL
#define PID_MAX_INTEGRAL 1000.0f
#endif

#define MANUAL    0
#define AUTOMATIC 1

struct PIDvalues {
    double kP, kI, kD, lowLimit, highLimit;
};
class PID{
private:
    double kP = 1.0f;
    double kI = 0.0f;
    double kD = 0.0f;
    double minDeadSpace = -DEADZONE;
    double maxDeadSpace = DEADZONE;
    double lowLimit = -1.0f;
    double highLimit = 1.0f;
    double previousError;
    double proportionalSteering;
    double integralSteering;
    double deriverativeSteering;
    double result, input, pidSteering;
    bool automaticMode = true;
public:
    PID(){}
    PID(double kP, double kI, double kD){
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
    }

    PID(PIDvalues values){
        this->kP = values.kP;
        this->kI = values.kI;
        this->kD = values.kD;
        this->lowLimit = values.lowLimit;
        this->highLimit = values.highLimit;
    }

    void setP(double kP){
        this->kP = kP;
    }

    void setI(double kI){
        this->kI = kI;
    }

    void setD(double kD){
        this->kD = kD;
    }

    double getP(){
        return proportionalSteering;
    }

    double getI(){
        return integralSteering;
    }

    double getD(){
        return deriverativeSteering;
    }

    double computeNewSteering(unsigned int timeDiff, double input){
        if(this->kP == 0.0f && this->kI == 0.0f && this->kD == 0.0f){
            return 1.0f;
        }
        this->input = input;

        double error = input - result;

        integralSteering += kI*error*(double)timeDiff;
        double errorDelta = previousError - error;

        proportionalSteering = kP*error;
        deriverativeSteering = kD*errorDelta;

        if(integralSteering >  PID_MAX_INTEGRAL) integralSteering = PID_MAX_INTEGRAL;
        if(integralSteering < -PID_MAX_INTEGRAL) integralSteering = -PID_MAX_INTEGRAL;

        pidSteering = proportionalSteering + integralSteering + deriverativeSteering;

        if(maxDeadSpace > pidSteering && pidSteering > 0) pidSteering = maxDeadSpace;
        if(minDeadSpace < pidSteering && pidSteering < 0) pidSteering = minDeadSpace;
        if(pidSteering > this->highLimit) pidSteering = this->highLimit;
        if(pidSteering < this->lowLimit) pidSteering = this->lowLimit;

        previousError = error;
        return pidSteering;
    }

    void setResult(double result){
        this->result = result;
    }

    void setMode(int mode){
        if(!automaticMode && mode == AUTOMATIC){
            reset();
            automaticMode = true;
        }else{
            automaticMode = mode == AUTOMATIC;
        }
    }

    void reset(){
        input = 0;
        previousError = 0;
        integralSteering = 0;
        result = 0;
        if(integralSteering >  PID_MAX_INTEGRAL) integralSteering = PID_MAX_INTEGRAL;
        if(integralSteering < -PID_MAX_INTEGRAL) integralSteering = -PID_MAX_INTEGRAL;
    }

};
#endif
