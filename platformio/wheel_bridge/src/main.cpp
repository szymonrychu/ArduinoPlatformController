#include "Arduino.h"
#include "pins.h"
#include "HardwareEncoder.h"
#include "Command.h"
#include "DistanceVelocityPID.h"
#include "SerialLogger.h"
#include "Hbridge.h"
#include "PID.h"

#ifndef MEM_LEN
#define MEM_LEN 256
#endif

char buffer[MEM_LEN];
long loopCounter = 0;
unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;

HardwareEncoder<1> angleEncoder;
DistanceVelocityPID anglePIDs;
Hbridge angle(22, 23);

HardwareEncoder<2> distanceEncoder;
DistanceVelocityPID distancePIDs;
Hbridge distance(20, 21);

float timeDelta = 0;

float angleTarget = 0;
float distanceTarget = 0;
float angleVelocityTarget = 0;
float distanceVelocityTarget = 0;

long distancePower = 0;
long anglePower = 0;

bool optoPrevState = false;
bool optoResetSuccessfull = false;

#define WHEEL_STATE_FRESH           0
#define WHEEL_STATE_RESET_REQUESTED 1
#define WHEEL_STATE_ACCEPTING_CMDS  2
#define WHEEL_STATE_BUSY            4

int wheelState = WHEEL_STATE_FRESH;

#include "commands.h"

void serialEvent() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        command.inputChar(ch);
    }
}

void reset(){
    wheelState = WHEEL_STATE_FRESH;
    angleEncoder.setup(ENCODER_TO_ANGLE);
    distanceEncoder.setup(ENCODER_TO_DISTANCE);
    angleEncoder.start();
    distanceEncoder.start();
    angleEncoder.zeroFTM();
    distanceEncoder.zeroFTM();
    anglePIDs.setup(ANGLE_DPID_P, ANGLE_DPID_I, ANGLE_DPID_D, ANGLE_VPID_P, ANGLE_VPID_I, ANGLE_VPID_D);
    distancePIDs.setup(DISTANCE_DPID_P, DISTANCE_DPID_I, DISTANCE_DPID_D, DISTANCE_VPID_P, DISTANCE_VPID_I, DISTANCE_VPID_D);
}

void setup(){
    memset(buffer, 0, MEM_LEN);
    Serial.begin(115200);
    setupCommands();
    angle.setup();
    distance.setup();
    while(Serial){;} // Wait for someone to open Serial port
}

void printDiagnostics(){
    sprintf(buffer, "%d %.4f:%.4f:%.4f:%d %.4f:%.4f:%.4f:%d %.3f", wheelState,
        angleEncoder.getLastPosition(), anglePIDs.getError(), angleEncoder.getLastVelocity()*1000.0*1000.0, anglePower,
        distanceEncoder.getLastPosition(), distancePIDs.getError(), distanceEncoder.getLastVelocity()*1000.0*1000.0, distancePower,
        timeDelta*1000.0);
    Logger::info(buffer);
}

void optoInterruptHandler(){
    if(optoPrevState && digitalRead(13) == LOW){
        angle.drive(0);
        detachInterrupt(digitalPinToInterrupt(13));
        reset();
        optoPrevState = false;
        wheelState = WHEEL_STATE_ACCEPTING_CMDS;
    }else if(!optoPrevState && digitalRead(13) == HIGH){
        optoPrevState = true;
    }
}

void loop(){
    unsigned long currentTime = micros();
    timeDelta = (float)(currentTime - lastLoopTime)/1000000.0;
    lastLoopTime = currentTime;

    switch(wheelState){
        case WHEEL_STATE_FRESH:
            angle.drive(pow(2, PWM_RESOLUTION)*ZEROING_RELATIVE_POWER);
            delay(100);
            attachInterrupt(digitalPinToInterrupt(13), optoInterruptHandler, CHANGE);
            wheelState = WHEEL_STATE_RESET_REQUESTED;
            break;
        case WHEEL_STATE_RESET_REQUESTED:
            delay(1);
            break;
        case WHEEL_STATE_ACCEPTING_CMDS:
        case WHEEL_STATE_BUSY:

            command.parse();

            angleEncoder.compute();
            distanceEncoder.compute();

            distancePower = distancePIDs.computeSteering(distanceTarget, distanceVelocityTarget, (timeDelta/1000.0));
            anglePower = anglePIDs.computeSteering(angleTarget, angleVelocityTarget, (timeDelta/1000.0));
            
            distance.drive(distancePower);
            angle.drive(anglePower);

            distancePIDs.setResults(distanceEncoder.getLastPosition(), distanceEncoder.getLastVelocity());
            anglePIDs.setResults(angleEncoder.getLastPosition(), angleEncoder.getLastVelocity());
            break;
        default:
            delay(1);
            break;
    }

    if(currentTime - lastPrintTime > 100.0*1000.0){
        printDiagnostics();
        lastPrintTime = currentTime;
    }
}
