#ifndef Controller_H
#define Controller_H

#include "DistAnglePIDDriver.h"

class RobotController {
private:
    DistAnglePIDDriver wheels[MOTOR_N];
public:
    RobotController(DistAnglePIDDriverConfig wheelsConfig[]){
        for(int c=0; c<MOTOR_N; c++){
            wheels[c] = DistAnglePIDDriver(wheelsConfig[c]);
        }
    }

    void computeAll(){
        for(int c=0; c<MOTOR_N; c++){
            wheels[c].setResults(LOOP_T);
        }
        delay(LOOP_T);
        for(int c=0; c<MOTOR_N; c++){
            wheels[c].compute();
        }
    }

    void handleDistanceInterrupt(int motorNumber){
        wheels[motorNumber].handleDistanceInterrupt();
    }

    void handleAngleInterrupt(int motorNumber){
        wheels[motorNumber].handleAngleInterrupt();
    }

    void inputAbsoluteDistanceVelocity(int motorNumber, double distance, double velocity){
        if(motorNumber >= MOTOR_N) return;
        if(motorNumber < 0) return;
        wheels[motorNumber].inputAbsoluteDistanceVelocity(distance, velocity);
    }

    void inputAbsoluteAngleVelocity(int motorNumber, double angle, double velocity){
        if(motorNumber >= MOTOR_N) return;
        if(motorNumber < 0) return;
        wheels[motorNumber].inputAbsoluteAngleVelocity(angle, velocity);
    }

    void reset(){
        for(int c=0; c<MOTOR_N; c++){
            wheels[c].reset();
        }
    }

    void printDiagnostics(){
        Serial.println("Diagnostics: ");
        for(int c=0; c<MOTOR_N; c++){
            Serial.print("Wheel");
            Serial.print(c+1);
            Serial.print(": ");
            wheels[c].printDiagnostics();
            Serial.println(";");
        }
        Serial.println(";");
    }

};
#endif


// void printDistanceDetails(){
//     Serial.print(distDriver.getDistance());
//     Serial.print("/");
//     Serial.print(inputDistance);
//     Serial.print("#");
//     Serial.print(distDriver.getDistSteering());
//     Serial.print("\t");
//     Serial.print(distDriver.getVelocity());
//     Serial.print("/");
//     Serial.print(inputVelocity);
//     Serial.print("#");
//     Serial.print(distDriver.getVeloSteering());
//     Serial.print("\t");
//     Serial.print(distDriver.getPower());
//     Serial.print("\t");
//     Serial.print("DPID:");
//     Serial.print(distDriver.getDistP());
//     Serial.print("/");
//     Serial.print(distDriver.getDistI());
//     Serial.print("/");
//     Serial.print(distDriver.getDistD());
//     Serial.print("\t");
//     Serial.print("VPID:");
//     Serial.print(distDriver.getVeloP());
//     Serial.print("/");
//     Serial.print(distDriver.getVeloI());
//     Serial.print("/");
//     Serial.print(distDriver.getVeloD());
// }
// void printAngleDetails(){
//     Serial.print(anglDriver.getDistance());
//     Serial.print("/");
//     Serial.print(inputDistance);
//     Serial.print("#");
//     Serial.print(anglDriver.getDistSteering());
//     Serial.print("\t");
//     Serial.print(anglDriver.getVelocity());
//     Serial.print("/");
//     Serial.print(inputVelocity);
//     Serial.print("#");
//     Serial.print(anglDriver.getVeloSteering());
//     Serial.print("\t");
//     Serial.print(anglDriver.getPower());
//     Serial.print("\t");
//     Serial.print("DPID:");
//     Serial.print(anglDriver.getDistP());
//     Serial.print("/");
//     Serial.print(anglDriver.getDistI());
//     Serial.print("/");
//     Serial.print(anglDriver.getDistD());
//     Serial.print("\t");
//     Serial.print("VPID:");
//     Serial.print(anglDriver.getVeloP());
//     Serial.print("/");
//     Serial.print(anglDriver.getVeloI());
//     Serial.print("/");
//     Serial.print(anglDriver.getVeloD());
// }


// void printDistDetails(){
//     printDistanceDetails();
// }

// int loopCounter1 = 0;
// int loopCounter2 = 800;
// int loopCounter3 = 1;
