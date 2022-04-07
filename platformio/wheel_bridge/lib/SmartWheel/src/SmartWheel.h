#ifndef SMART_WHEEL_H
#define SMART_WHEEL_H

#include "Arduino.h"
#include "SmartWheel_statics.h"
#include "MoveRequest.h"

#include "HardwareEncoder.h"
#include "DistanceVelocityPID.h"
#include "Hbridge.h"

#define ZERO_DEG_OPTO_PIN 13
#define DISTANCE_H_BRIDGE_A 20
#define DISTANCE_H_BRIDGE_B 21
#define ANGLE_H_BRIDGE_A 22
#define ANGLE_H_BRIDGE_B 23

#define WHEEL_STATE_FRESH           0
#define WHEEL_STATE_RESET_REQUESTED 1
#define WHEEL_STATE_ACCEPTING_CMDS  2
#define WHEEL_STATE_BUSY            4


class SmartWheel;
SmartWheel * instance_;

class SmartWheel {
private:
    char buffer[MEM_LEN];
    unsigned long lastLoopTime = 0;
    float timeDelta = 0;
    uint8_t currentState = WHEEL_STATE_FRESH;

    float angleTarget = 0;
    float angleVelocityTarget = 0;
    long anglePower = 0;
    HardwareEncoder<1> angleEncoder;
    DistanceVelocityPID anglePIDs;
    Hbridge angleHBridge;
    bool angleReached = true;


    float distanceTarget = 0;
    float distanceVelocityTarget = 0;
    long distancePower = 0;
    HardwareEncoder<2> distanceEncoder;
    DistanceVelocityPID distancePIDs;
    Hbridge distanceHBridge;
    bool distanceReached = true;

    void (*zeroDegreesHook)(float) = NULL;
    bool optoPrevState = false;
    void zeroDegreesHandler(){
        float lastAngleRad = this->angleEncoder.getLastPosition();
        if(this->currentState == WHEEL_STATE_RESET_REQUESTED){
            if(optoPrevState && digitalRead(ZERO_DEG_OPTO_PIN) == LOW){
                angleHBridge.drive(0);
                zeroEncoders();
                optoPrevState = false;
                this->currentState = WHEEL_STATE_ACCEPTING_CMDS;
            }else if(!optoPrevState && digitalRead(ZERO_DEG_OPTO_PIN) == HIGH){
                optoPrevState = true;
            }
        }
        if(zeroDegreesHook != NULL && lastAngleRad > PI/4 && digitalRead(ZERO_DEG_OPTO_PIN) == HIGH){
            zeroDegreesHook(lastAngleRad);
        }
    }

    void doReset(){
        angleHBridge.drive(pow(2, PWM_RESOLUTION)*ZEROING_RELATIVE_POWER);
        delay(100);
        attachInterrupt(digitalPinToInterrupt(ZERO_DEG_OPTO_PIN), zeroDegreesHandler_, CHANGE);
    }

public:
    static void zeroDegreesHandler_(){
        instance_->zeroDegreesHandler();
    }

    SmartWheel(){
        memset(buffer, 0, MEM_LEN);
        instance_ = reinterpret_cast<SmartWheel*>(this);

        angleHBridge = Hbridge(ANGLE_H_BRIDGE_A, ANGLE_H_BRIDGE_B);
        anglePIDs.setup(ANGLE_DPID_P, ANGLE_DPID_I, ANGLE_DPID_D, ANGLE_VPID_P, ANGLE_VPID_I, ANGLE_VPID_D);
        angleEncoder.setup(ENCODER_TO_ANGLE);
        angleEncoder.start();

        distanceHBridge = Hbridge(DISTANCE_H_BRIDGE_A, DISTANCE_H_BRIDGE_B);
        distancePIDs.setup(DISTANCE_DPID_P, DISTANCE_DPID_I, DISTANCE_DPID_D, DISTANCE_VPID_P, DISTANCE_VPID_I, DISTANCE_VPID_D);
        distanceEncoder.setup(ENCODER_TO_DISTANCE);
        distanceEncoder.start();
        zeroEncoders();
    }

    void zeroEncoders(){
        angleEncoder.zeroFTM();
        distanceEncoder.zeroFTM();
        angleReached = true;
        distanceReached = true;
        distanceTarget = 0;
        distanceVelocityTarget = 0;
        distancePower = 0;
        angleTarget = 0;
        angleVelocityTarget = 0;
        anglePower = 0;
        distanceReached = false;
        angleReached = false;
    }

    uint8_t getCurrentState(){
        return this->currentState;
    }

    bool isBusy(){
        return this->currentState == WHEEL_STATE_BUSY && (!angleReached || !distanceReached);
    }

    void compute(){
        unsigned long currentTime = micros();
        timeDelta = (float)(currentTime - lastLoopTime)/1000000.0;
        lastLoopTime = currentTime;
        switch(getCurrentState()){
            case WHEEL_STATE_FRESH:
                requestReset();
                break;
            case WHEEL_STATE_RESET_REQUESTED:
                break;
            case WHEEL_STATE_ACCEPTING_CMDS:
            case WHEEL_STATE_BUSY:
                angleEncoder.compute(timeDelta);
                distanceEncoder.compute(timeDelta);

                distancePower = distancePIDs.computeSteering(distanceTarget, distanceVelocityTarget, timeDelta);
                anglePower = anglePIDs.computeSteering(angleTarget, angleVelocityTarget, timeDelta);

                distanceReached = !distanceHBridge.drive(distancePower);
                angleReached = !angleHBridge.drive(anglePower);

                distancePIDs.setResults(distanceEncoder.getLastPosition(), distanceEncoder.getLastVelocity());
                anglePIDs.setResults(angleEncoder.getLastPosition(), angleEncoder.getLastVelocity());

                
                // if(!distanceReached){
                //     distanceReached = abs(distanceEncoder.getLastPosition() - this->distanceTarget) < DISTANCE_DONE_SENSITIVITY;
                // }
                // if(!angleReached){
                //     angleReached = abs(angleEncoder.getLastPosition() - this->angleTarget) < ANGLE_DONE_SENSITIVITY;
                // }

                if(distanceReached && angleReached){
                    this->currentState = WHEEL_STATE_ACCEPTING_CMDS;
                }
                break;
            default:
                this->currentState = WHEEL_STATE_FRESH;
        }
        while(timeDelta < 0.001){
            timeDelta += 10.0/1000000.0;
            delayMicroseconds(10);
        }
    }

    void requestReset(){
        this->currentState = WHEEL_STATE_RESET_REQUESTED;
        doReset();
    }

    char* requestMove(MoveRequest m){
        if(m.isDistanceSet()){
            this->distanceTarget = m.getRelativeDistanceM(this->distanceEncoder.getLastPosition());
            this->distanceVelocityTarget = m.getDistanceVelocity(this->distanceEncoder.getLastPosition());
            distanceReached = false;
        }
        if(m.isAngleSet()){
            this->angleTarget = m.getAbsoluteAngleRad();
            this->angleVelocityTarget = m.getAngleVelocity(this->angleEncoder.getLastPosition());
            angleReached = false;
        }
        this->currentState = WHEEL_STATE_BUSY;
        
        sprintf(buffer, "requestMove:%.4f:%.4f:%.4f:%.4f", 
            this->distanceTarget, this->distanceVelocityTarget,
            this->angleTarget, this->angleVelocityTarget);
        return buffer;
    }

    char* getDiagnostics(){
        sprintf(buffer, "%d %.4f:%.4f:%.4f %d %.4f:%.4f:%.4f %d %.4f", this->currentState,
            distanceEncoder.getLastPosition(), distancePIDs.getError(), distancePIDs.getVelocityError(), distancePower,
            angleEncoder.getLastPosition(), anglePIDs.getError(), anglePIDs.getVelocityError(), anglePower,
            timeDelta*1000.0);
        return buffer;
    }

    void attachZeroRadHook(void (zeroDegreesHook)(float)){
        this->zeroDegreesHook = zeroDegreesHook;
    }

    void detachZeroRadHook(){
        this->zeroDegreesHook = NULL;
    }

};

#endif