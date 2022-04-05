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
    float relativeDistanceM = 0;
    float absoluteAngleRad = 0;
    float timeToReachS = 0;
    bool distanceSet = false;
    bool angleSet = false;
public:
    void setRelativeDistanceM(float d){
        this->relativeDistanceM = d;
        this->distanceSet = true;
    }
    void setAbsoluteAngleRad(float a){
        this->absoluteAngleRad = a;
        this->angleSet = true;
    }
    void setTimeS(float t){
        this->timeToReachS = t;
    }

    float getTimeS(){
        return this->timeToReachS;
    }

    bool isDistanceSet(){
        return this->distanceSet;
    }
    float getRelativeDistanceM(float currentDistanceM=0){
        if(this->distanceSet){
            return this->relativeDistanceM + currentDistanceM;
        }else{
            return currentDistanceM;
        }
    }
    float getDistanceVelocity(float currentDistanceM=0){
        if(this->distanceSet){
            return abs(getRelativeDistanceM())/getTimeS();
        }else{
            return 0.0f;
        }
    }

    bool isAngleSet(){
        return this->angleSet;
    }
    float getAbsoluteAngleRad(float currentAngleRad=0){
        if(this->angleSet){
            return this->absoluteAngleRad - currentAngleRad;
        }else{
            return currentAngleRad;
        }
    }
    float getAngleVelocity(float currentAngleRad=0){
        if(this->angleSet){
            return abs(getAbsoluteAngleRad(currentAngleRad))/getTimeS();
        }else{
            return 0.0f;
        }
    }

    bool parseRequest(Command& command){

        char* distanceCH = command.next();
        float d = atof(distanceCH);
        char* angleCH = command.next();
        float a = atof(angleCH);
        char* timeS = command.next();
        float t = atof(timeS);

        if(t > 0){
            if(distanceCH != NULL){
                setRelativeDistanceM(d);
            }
            if(angleCH != NULL){
                setAbsoluteAngleRad(a);
            }
            setTimeS(t);
            return true;
        }
        return false;
    }
};

#endif