#ifndef DISTANCEVELOCITYPID_h
#define DISTANCEVELOCITYPID_h

#ifndef MIN_VELOCITY_PID
#define MIN_VELOCITY_PID 0.0
#endif

#include "PID.h"

class DistanceVelocityPID{
private:
    PID distancePID;
    PID velocityPID;
	unsigned long latestTimestampMicros;
    double distanceSteering = 0;
    double velocitySteering = 0;
public:
    void setup(double dP, double dI, double dD, double vP, double vI, double vD){
        this->distancePID = PID(dP, dI, dD);
        this->velocityPID = PID(vP, vI, vD, MIN_VELOCITY_PID, 1.0);
        this->latestTimestampMicros = 0;
    }

    long computeSteering(double dInput, double vInput){
        if(latestTimestampMicros==0){
            latestTimestampMicros = micros();
            return 0.0;
        }
        double timeDelta = (double)(latestTimestampMicros - micros())/1000000.0;
        return computeSteering(dInput, vInput, timeDelta);
    }

    double getError(){
        return this->distancePID.getError();
    }

    double getVelocityError(){
        return this->velocityPID.getError();
    }

    long computeSteering(double dInput, double vInput, double timeDelta){
        distanceSteering = this->distancePID.computeNewSteering(timeDelta, dInput);
        velocitySteering = this->velocityPID.computeNewSteering(timeDelta, vInput);
        long drive = (long)((double)pow(2, PWM_RESOLUTION) * distanceSteering * velocitySteering);
        if(drive > pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER){
            drive = pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER;
        }
        if(drive < -pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER){
            drive = -pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER;
        }
        return drive;
    }

    double getDistanceSteering(){
        return this->distanceSteering;
    }

    double getVelocitySteering(){
        return this->velocitySteering;
    }


    void setResults(double dResult, double vResult){
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