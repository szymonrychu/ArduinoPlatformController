#ifndef DISTANCEVELOCITYPID_h
#define DISTANCEVELOCITYPID_h

#include "PID.h"

class DistanceVelocityPID{
private:
    PID distancePID;
    PID velocityPID;
	unsigned long latestTimestampMicros;
public:
    void setup(float dP, float dI, float dD, float vP, float vI, float vD){
        this->distancePID = PID(dP, dI, dD);
        this->velocityPID = PID(vP, vI, vD, 0.0, 1.0);
        this->latestTimestampMicros = 0;
    }

    long computeSteering(float dInput, float vInput){
        if(latestTimestampMicros==0){
            latestTimestampMicros = micros();
            return 0.0;
        }
        float timeDelta = (float)(latestTimestampMicros - micros())/1000000.0;
        return computeSteering(dInput, vInput, timeDelta);
    }

    float getError(){
        return this->distancePID.getError();
    }

    float getVelocityError(){
        return this->velocityPID.getError();
    }

    long computeSteering(float dInput, float vInput, float timeDelta){
        float distanceSteering = this->distancePID.computeNewSteering(timeDelta, dInput);
        float velocitySteering = this->velocityPID.computeNewSteering(timeDelta, vInput);
        long drive = (long)((float)pow(2, PWM_RESOLUTION) * distanceSteering * velocitySteering);
        if(drive > pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER){
            drive = pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER;
        }
        if(drive < -pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER){
            drive = -pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER;
        }
        return drive;
    }


    void setResults(float dResult, float vResult){
        this->distancePID.setResult(dResult);
        this->velocityPID.setResult(vResult);
    }

    PIDvalues getPIDValues(){
        return distancePID.getPIDValues();
    }

    PIDvalues getVelocityPIDValues(){
        return velocityPID.getPIDValues();
    }

    void setPidValues(PIDvalues data){
        distancePID.setPIDValues(data);
    }

    void setVelocityPidValues(PIDvalues data){
        velocityPID.setPIDValues(data);
    }
};

#endif