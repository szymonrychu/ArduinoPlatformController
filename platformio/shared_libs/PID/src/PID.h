#ifndef PID_H
#define PID_H

#ifndef PID_MAX_INTEGRAL
#define PID_MAX_INTEGRAL 1000.0f
#endif

#define MANUAL    0
#define AUTOMATIC 1

struct PIDvalues {
    float kP, kI, kD, lowLimit, highLimit;
};
class PID{
private:
    float kP = 1.0f;
    float kI = 0.0f;
    float kD = 0.0f;
    // float minDeadSpace = -DEADZONE;
    // float maxDeadSpace = DEADZONE;
    float lowLimit = -1.0f;
    float highLimit = 1.0f;
    float previousError;
    float proportionalSteering;
    float integralSteering;
    float deriverativeSteering;
    float result, input, pidSteering;
    float error = 0.0f;
    bool automaticMode = true;
public:
    PID(){}
    PID(float kP, float kI, float kD, float lowLimit, float highLimit){
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
        this->lowLimit = lowLimit;
        this->highLimit = highLimit;
    }
    PID(float kP, float kI, float kD){
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
    float getError(){
        return this->error;
    }

    PIDvalues getPIDValues(){
        PIDvalues data = {this->kP, this->kI, this->kD, this->lowLimit, this->highLimit};
        return data;
    }

    void setPIDValues(PIDvalues data){
        this->kP = data.kP;
        this->kI = data.kI;
        this->kD = data.kD;
        this->lowLimit = data.lowLimit;
        this->highLimit = data.highLimit;
    }

    float computeNewSteering(float timeDiff, float input){
        if(this->kP == 0.0f && this->kI == 0.0f && this->kD == 0.0f){
            return 1.0f;
        }
        this->input = input;

        error = this->input - this->result;
        // if(abs(error) < PID_DELTA_TOLERANCE){
        //     error = 0;
        // }

        this->integralSteering += kI*error*timeDiff;
        float errorDelta = (this->previousError - error)/timeDiff;

        proportionalSteering = kP*error;
        deriverativeSteering = kD*errorDelta;

        if(this->integralSteering >  PID_MAX_INTEGRAL) this->integralSteering = PID_MAX_INTEGRAL;
        if(this->integralSteering < -PID_MAX_INTEGRAL) this->integralSteering = -PID_MAX_INTEGRAL;

        pidSteering = proportionalSteering + this->integralSteering + deriverativeSteering;

        // if(maxDeadSpace > pidSteering && pidSteering > 0) pidSteering = maxDeadSpace;
        // if(minDeadSpace < pidSteering && pidSteering < 0) pidSteering = minDeadSpace;
        if(pidSteering > this->highLimit) pidSteering = this->highLimit;
        if(pidSteering < this->lowLimit) pidSteering = this->lowLimit;

        this->previousError = error;
        return pidSteering;
    }

    void setResult(float result){
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

    void reset(float currentDistance=0.0f){
        this->input = currentDistance;
        this->previousError = 0;
        this->integralSteering = 0;
        this->result = 0;
    }

};
#endif
