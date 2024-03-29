#ifndef SMART_WHEEL_H
#define SMART_WHEEL_H

#include "Arduino.h"
#include "RobotSimplified_statics.h"
#include "MoveRequest.h"

#include "HardwareEncoder.h"
#include "DistanceVelocityPID.h"
#include "Hbridge.h"

#define LEFT_PWM_PIN 22
#define LEFT_PIN_A   20
#define LEFT_PIN_B   14

#define RIGHT_PWM_PIN 23
#define RIGHT_PIN_A   21
#define RIGHT_PIN_B   15

#define ROBOT_STATE_READY 0
#define ROBOT_STATE_BUSY_MOVING_FORWARD 1
#define ROBOT_STATE_BUSY_MOVING_BACKWARD 2
#define ROBOT_STATE_BUSY_MOVING_TURNING 3

#define E2I_F 0.1


class RobotSimplified;
RobotSimplified * instance_;

class RobotSimplified {
private:
    char buffer[MEM_LEN];
    unsigned long lastLoopTime = 0;
    float timeDelta = 0;
    uint8_t state = ROBOT_STATE_READY;

    float leftTarget = 0;
    float leftVelocityTarget = 0;
    float lastLeftPosition = 0;
    long leftPower = 0;
    long previousLeftPower = 0;
    HardwareEncoder<1> leftEncoder;
    DistanceVelocityPID leftPIDs;
    Hbridge leftHBridge;
    bool leftReached = true;
    float deltaLeft = 0;

    float rightTarget = 0;
    float rightVelocityTarget = 0;
    float lastRightPosition = 0;
    long rightPower = 0;
    long previousRightPower = 0;
    HardwareEncoder<2> rightEncoder;
    DistanceVelocityPID rightPIDs;
    Hbridge rightHBridge;
    bool rightReached = true;
    float deltaRight = 0;

public:

    RobotSimplified(){
        memset(buffer, 0, MEM_LEN);
        instance_ = reinterpret_cast<RobotSimplified*>(this);

        leftHBridge = Hbridge(LEFT_PWM_PIN, LEFT_PIN_A, LEFT_PIN_B);
        leftPIDs.setup(DISTANCE_DPID_P, DISTANCE_DPID_I, DISTANCE_DPID_D, DISTANCE_VPID_P, DISTANCE_VPID_I, DISTANCE_VPID_D);
        leftEncoder.setup(ENCODER_TO_DISTANCE);
        leftEncoder.start();

        rightHBridge = Hbridge(RIGHT_PWM_PIN, RIGHT_PIN_A, RIGHT_PIN_B);
        rightPIDs.setup(DISTANCE_DPID_P, DISTANCE_DPID_I, DISTANCE_DPID_D, DISTANCE_VPID_P, DISTANCE_VPID_I, DISTANCE_VPID_D);
        rightEncoder.setup(ENCODER_TO_DISTANCE);
        rightEncoder.start();
    }
    
    #define DIFFERENTIAL_P 0.001
    
    void compute(){
        unsigned long currentTime = micros();
        timeDelta = (float)(currentTime - lastLoopTime)/1000000.0;
        if(timeDelta < 0.01){
            return;
        }
        lastLoopTime = currentTime;

        float latestLeftPosition = leftEncoder.getLastPosition();
        float latestRightPosition = rightEncoder.getLastPosition();
        
        leftEncoder.compute(timeDelta);
        rightEncoder.compute(timeDelta);

        this->leftPower = leftPIDs.computeSteering(leftTarget, leftVelocityTarget, timeDelta);
        this->rightPower = rightPIDs.computeSteering(rightTarget, rightVelocityTarget, timeDelta);

        leftHBridge.drive(leftPower);
        rightHBridge.drive(rightPower);

        leftPIDs.setResults(latestLeftPosition, leftEncoder.getLastVelocity());
        rightPIDs.setResults(latestRightPosition, rightEncoder.getLastVelocity());


        // float differentialError = this->leftPIDs.getError() - this->rightPIDs.getError();
        // if(this->state == ROBOT_STATE_BUSY_MOVING_FORWARD){
        //     leftVelocityTarget = leftVelocityTarget + DIFFERENTIAL_P * differentialError;
        //     rightVelocityTarget = rightVelocityTarget - DIFFERENTIAL_P * differentialError;
        // }else if(this->state == ROBOT_STATE_BUSY_MOVING_BACKWARD){
        //     leftVelocityTarget = leftVelocityTarget - DIFFERENTIAL_P * differentialError;
        //     rightVelocityTarget = rightVelocityTarget + DIFFERENTIAL_P * differentialError;
        // }

        // while(timeDelta < 0.01){
        //     timeDelta += 10.0/1000000.0;
        // }
        // delayMicroseconds(100);
    }

    char* requestMove(MoveRequest m){
        if(m.isLeftSet()){
            this->leftTarget = m.getRelativeLeftDistance(E2I_F*this->leftEncoder.getLastPosition())/E2I_F;
            this->leftVelocityTarget = m.getRelativeLeftVelocity()/E2I_F;
        }
        if(m.isRightSet()){
            this->rightTarget = m.getRelativeRightDistance(E2I_F*this->rightEncoder.getLastPosition())/E2I_F;
            this->rightVelocityTarget = m.getRelativeRightVelocity()/E2I_F;
        }
        if(m.isMovingForward()){
            this->state = ROBOT_STATE_BUSY_MOVING_FORWARD;
        }else if(m.isMovingBackward()){
            this->state = ROBOT_STATE_BUSY_MOVING_BACKWARD;
        }else if (m.isTurning()){
            this->state = ROBOT_STATE_BUSY_MOVING_TURNING;
        }
        
        sprintf(buffer, "requestMove:%.4f:%.4f:%.4f:%.4f", 
            m.getRelativeLeftDistance(), this->leftVelocityTarget,
            m.getRelativeRightDistance(), this->rightVelocityTarget);
        return buffer;
    }

    char* getDiagnostics(){
        float left = E2I_F*leftEncoder.getLastPosition();
        deltaLeft = left - lastLeftPosition;
        lastLeftPosition = left;

        float right = E2I_F*rightEncoder.getLastPosition();
        deltaRight = right - lastRightPosition;
        lastRightPosition = right;

        sprintf(buffer, "%d:%ld:%ld:%.4f:%.4f:%.4f:%.4f",
            this->state, leftPower, rightPower,
            left, right, leftPIDs.getError(), rightPIDs.getError());
        return buffer;
    }

    bool isBusy(){
        return this->state == ROBOT_STATE_BUSY_MOVING_FORWARD || this->state == ROBOT_STATE_BUSY_MOVING_BACKWARD || this->state == ROBOT_STATE_BUSY_MOVING_TURNING;
    }

};

#endif