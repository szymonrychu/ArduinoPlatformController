#ifndef MOVE_REQUEST_H
#define MOVE_REQUEST_H

#include "Arduino.h"
#include "Command.h"

#define MOVE_REQUEST_TYPE_RESET 0
#define MOVE_REQUEST_TYPE_MOVE  1

class Request {
public:
    virtual bool parseRequest(Command&command) = 0;

};

class ResetRequest: public Request {

};

class MoveRequest: public Request {
private:
    float relativeLeftDistance = 0;
    float relativeRightDistance = 0;
    float timeToReachS = 0;
    bool movingForward = false;
    bool movingBackward = false;
    bool turning = false;
    bool leftSet = false;
    bool rightSet = false;
public:
    void setRelativeLeftM(float d){
        this->relativeLeftDistance = d;
        this->leftSet = true;
    }
    void setRelativeRightM(float d){
        this->relativeRightDistance = d;
        this->rightSet = true;
    }
    void setTimeS(float t){
        this->timeToReachS = t;
    }

    float getTimeS(){
        return this->timeToReachS;
    }

    bool isLeftSet(){
        return this->leftSet;
    }
    float getRelativeLeftDistance(float currentLeftM=0){
        if(this->isLeftSet()){
            return this->relativeLeftDistance + currentLeftM;
        }else{
            return currentLeftM;
        }
    }
    float getRelativeLeftVelocity(float currentLeftM=0){
        if(this->isLeftSet()){
            return abs(getRelativeLeftDistance())/getTimeS();
        }else{
            return 0.0f;
        }
    }

    bool isRightSet(){
        return this->rightSet;
    }
    float getRelativeRightDistance(float currentRightM=0){
        if(this->isRightSet()){
            return this->relativeRightDistance + currentRightM;
        }else{
            return currentRightM;
        }
    }
    float getRelativeRightVelocity(float currentRightM=0){
        if(this->isRightSet()){
            return abs(getRelativeRightDistance())/getTimeS();
        }else{
            return 0.0f;
        }
    }

    bool isTurning(){
        return this->turning;
    }

    bool isMovingForward(){
        return this->movingForward;
    }

    bool isMovingBackward(){
        return this->movingBackward;
    }

    bool parseRequest(Command& command){

        char* leftCH = command.next();
        float l = atof(leftCH);
        char* rightCH = command.next();
        float r = atof(rightCH);
        char* timeS = command.next();
        float t = atof(timeS);

        if(t > 0){
            if(leftCH != NULL){
                setRelativeLeftM(l);
            }
            if(rightCH != NULL){
                setRelativeRightM(r);
            }
            if(this->getRelativeLeftDistance() == this->getRelativeRightDistance()){
                this->movingForward = this->getRelativeLeftDistance() > 0;
                this->movingBackward = !this->movingForward;
                this->turning = false;
            }else if(this->isLeftSet() || this->isRightSet()){
                this->movingForward = false;
                this->movingBackward = false;
                this->turning = true;
            }
            setTimeS(t);
            return true;
        }
        return false;
    }
};

#endif