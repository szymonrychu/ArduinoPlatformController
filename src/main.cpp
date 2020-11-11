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

HardwareEncoder<1> angleEncoder;
DistanceVelocityPID anglePIDs;
Hbridge angle(22, 23);

HardwareEncoder<2> distanceEncoder;
DistanceVelocityPID distancePIDs;
Hbridge distance(20, 21);

float timeDelta = 0;
float angleDelta = 0;
float distanceDelta = 0;
float angleVelocity = 0;
float distanceVelocity = 0;
float angleCurrentTarget = 0;
float distanceCurrentTarget = 0;
unsigned long targetLoops = 0;
long distancePower = 0;
long anglePower = 0;
bool optoPrevState = false;
bool optoResetSuccessfull = false;

#include "commands.h"

void serialEvent() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        command.inputChar(ch);
    }
}

void reset(){
    angleEncoder.setup(ENCODER_TO_ANGLE);
    distanceEncoder.setup(ENCODER_TO_DISTANCE);
    angleEncoder.start();
    distanceEncoder.start();
    angleEncoder.zeroFTM();
    distanceEncoder.zeroFTM();
    anglePIDs.setup(ANGLE_DPID_P, ANGLE_DPID_I, ANGLE_DPID_D, ANGLE_VPID_P, ANGLE_VPID_I, ANGLE_VPID_D);
    distancePIDs.setup(DISTANCE_DPID_P, DISTANCE_DPID_I, DISTANCE_DPID_D, DISTANCE_VPID_P, DISTANCE_VPID_I, DISTANCE_VPID_D);
    optoResetSuccessfull = true;
}

void optoInterruptHandler(){
    if(optoPrevState && digitalRead(13) == LOW){
        angle.drive(0);
        detachInterrupt(digitalPinToInterrupt(13));
        reset();
        optoPrevState = false;
    }else if(!optoPrevState && digitalRead(13) == HIGH){
        optoPrevState = true;
    }
}

void requestAngleZeroing(){
    angle.drive(pow(2, PWM_RESOLUTION)*MAX_RELATIVE_POWER);
    delay(100);
    optoResetSuccessfull = false;
    attachInterrupt(digitalPinToInterrupt(13), optoInterruptHandler, CHANGE);
}

void setup(){
    memset(buffer, 0, MEM_LEN);
    Serial.begin(115200);
    setupCommands();
    angle.setup();
    distance.setup();
    while(Serial){;} // Wait for someone to open Serial port
    requestAngleZeroing();
}

void printDiagnostics(){
    sprintf(buffer, "%.5f:%.5f:%.5f:%d %.5f:%.5f:%.5f:%d %.3f", 
        angleEncoder.getLastPosition(), angleEncoder.getLastVelocity(), anglePIDs.getError(), anglePower,
        distanceEncoder.getLastPosition(), distanceEncoder.getLastVelocity(), distancePIDs.getError(), distancePower,
        timeDelta*1000.0);
    Logger::info(buffer);
}

void loop(){
    if(optoResetSuccessfull){
        unsigned long currentTime = micros();
        timeDelta = (float)(currentTime - lastLoopTime)/1000000.0;
        lastLoopTime = currentTime;
        loopCounter = (loopCounter+1)%(LOOPS_TO_SECONDS*10); // longest loop will take up to 10s
        command.parse();

        angleEncoder.compute();
        distanceEncoder.compute();

        distancePower = distancePIDs.computeSteering(distanceCurrentTarget, distanceVelocity, timeDelta);
        anglePower = anglePIDs.computeSteering(angleCurrentTarget, angleVelocity, timeDelta);
        
        distance.drive(distancePower);
        angle.drive(anglePower);

        if(targetLoops>0){
            angleCurrentTarget += angleDelta;
            distanceCurrentTarget += distanceDelta;
            targetLoops--;
        }else{
            if(abs(angleCurrentTarget - angleEncoder.getLastPosition()) < angleDelta){
                angleVelocity = 0.0;
            }
            if(abs(distanceCurrentTarget - distanceEncoder.getLastPosition()) < distanceDelta){
                distanceVelocity = 0.0;
            }
        }

        if(targetLoops>0){
            printDiagnostics();
        }else if(millis()  % 1000 == 0){
            printDiagnostics();
        }

        distancePIDs.setResults(distanceEncoder.getLastPosition(), distanceEncoder.getLastVelocity());
        anglePIDs.setResults(angleEncoder.getLastPosition(), angleEncoder.getLastVelocity());
    }else{
        delay(1);
    }
}
