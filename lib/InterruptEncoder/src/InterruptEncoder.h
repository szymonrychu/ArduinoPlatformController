#include "Arduino.h"
#include "pins.h"

#ifndef INTERRUPT_ENCODER_h
#define INTERRUPT_ENCODER_h

#ifndef ENCODER_TOLERANCE
#define ENCODER_TOLERANCE -0.1
#endif

struct InterruptEncoderSetup{
    int pinA, pinB;
    double stepToDistance;
    // void (*interruptFunction)(void);
};

class InterruptEncoder {
private:
    int pinA = 0;
    int pinB = 0;
    double stepToDistance = 1.0f;
    void (*interruptFunction)(bool) = NULL;
    void (*debugFunction)(int, int, int, int) = NULL;
    int previousPinAState = LOW;
    int previousPinBState = LOW;
    double distance = 0.0f;
    double previousDistance = 0.0f;
    double distanceToReach = 0.0f;
    bool initialized = false;
    bool reached = true;
    bool corrIncrDistance = false;
    bool corrDecrDistance = false;
    double velocityPreviousDistance = 0.0f;
public:
    InterruptEncoder(){}

    InterruptEncoder(InterruptEncoderSetup setup){
        this->pinA = setup.pinA;
        this->pinB = setup.pinB;
        this->stepToDistance = setup.stepToDistance;
    }

    InterruptEncoder(InterruptEncoderSetup setup, void (interruptFunction(bool))){
        this->pinA = setup.pinA;
        this->pinB = setup.pinB;
        this->interruptFunction = interruptFunction;
        this->stepToDistance = setup.stepToDistance;
    }

    void setup(){
        pinMode(this->pinA, INPUT);
        pinMode(this->pinB, INPUT);
        // attachInterrupt(digitalPinToInterrupt(this->pinA), interruptFunction, CHANGE);
        // attachInterrupt(digitalPinToInterrupt(this->pinB), interruptFunction, CHANGE);
    }

    void attachDebugSerial(void (*debugFunction)(int, int, int, int)){
        this->debugFunction = debugFunction;
    }
    void handleInterrupt(){
        int pinAState = digitalRead(pinA);
        int pinBState = digitalRead(pinB);
        if(initialized) {
            /*
            pinA | pinB | prevA | prevB
            HIGH | LOW  | LOW   | LOW
            HIGH | HIGH | HIGH  | LOW
            LOW  | HIGH | HIGH  | HIGH
            LOW  | LOW  | LOW   | HIGH
            */
            if( (pinAState == HIGH && pinBState == LOW  && previousPinAState == LOW  && previousPinBState == LOW ) || \
                (pinAState == HIGH && pinBState == HIGH && previousPinAState == HIGH && previousPinBState == LOW ) || \
                (pinAState == LOW  && pinBState == HIGH && previousPinAState == HIGH && previousPinBState == HIGH) || \
                (pinAState == LOW  && pinBState == LOW  && previousPinAState == LOW  && previousPinBState == HIGH)
            ){
                previousDistance = distance;
                distance += DIR_DIST*stepToDistance;
            }
            /*
            pinA | pinB | prevA | prevB
            LOW  | LOW  | HIGH  | LOW
            LOW  | HIGH | LOW   | LOW
            HIGH | HIGH | LOW   | HIGH
            HIGH | LOW  | HIGH  | HIGH
            */
            else if( (pinAState == LOW  && pinBState == LOW  && previousPinAState == HIGH && previousPinBState == LOW ) || \
                (pinAState == LOW  && pinBState == HIGH && previousPinAState == LOW  && previousPinBState == LOW ) || \
                (pinAState == HIGH && pinBState == HIGH && previousPinAState == LOW  && previousPinBState == HIGH) || \
                (pinAState == HIGH && pinBState == LOW  && previousPinAState == HIGH && previousPinBState == HIGH)
            ){
                previousDistance = distance;
                distance -= DIR_DIST*stepToDistance;
            }else{
                if(debugFunction != NULL){
                    debugFunction(pinAState, pinBState, previousPinAState, previousPinBState);
                }
            }
        }
        previousPinAState = pinAState;
        previousPinBState = pinBState;
        if(!initialized) initialized = true;
        bool incrDistance = distance > previousDistance; //checks if we are going forward
        bool decrDistance = distance < previousDistance; //checks if we are going backward
        if(corrIncrDistance && incrDistance){
            // we correctly go forward
            bool incrReachedDistance = distance+ENCODER_TOLERANCE > distanceToReach; //checks if we reached target if going forward
            if(incrReachedDistance){
                reached = true;
            }
        }else if(corrDecrDistance && decrDistance){
            // we correctly go backward
            bool decrReachedDistance = distance-ENCODER_TOLERANCE < distanceToReach; //checks if we reached target if going backward
            if(decrReachedDistance){
                reached = true;
            }
        }
    }

    void handleEncoder(){
        if(reached && interruptFunction != NULL){
            interruptFunction(true);
            reached = false;
        }
    }

    double getDistance(){
        return distance;
    }

    void setCallbackFunction(void (interruptFunction(bool))){
        this->interruptFunction = interruptFunction;
    }

    double getVelocity(double timeDelta){
        double result = abs((distance-velocityPreviousDistance)/timeDelta);
        velocityPreviousDistance = distance;
        return result;
    }

    void setDistanceToReach(double distance){
        corrIncrDistance = distance > this->distance;
        corrDecrDistance = distance < this->distance;
        this->distanceToReach = distance;
    }

    void reset(double distanceToReach=0.0f){
        corrIncrDistance = false;
        corrDecrDistance = false;
        reached = false;
        this->distanceToReach = distanceToReach;
        distance = distanceToReach;
    }
};
#endif
