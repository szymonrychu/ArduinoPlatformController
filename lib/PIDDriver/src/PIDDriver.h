#ifndef PIDDriver_H
#define PIDDriver_H
#include "PID.h"
#include "pins.h"
#include "InterruptEncoder.h"
#include "PololuVNH3SP30.h"

class PIDConfig {
public:
    double distanceP;
    double distanceI;
    double distanceD;
    bool distancePdefined;
    bool distanceIdefined;
    bool distanceDdefined;
    
    double velocityP;
    double velocityI;
    double velocityD;
    bool velocityPdefined;
    bool velocityIdefined;
    bool velocityDdefined;

    PIDConfig(){
        distancePdefined = false;
        distanceIdefined = false;
        distanceDdefined = false;
        velocityPdefined = false;
        velocityIdefined = false;
        velocityDdefined = false;
    }
};

class PIDDriver {
private:
    PID velocityPID;
    PID distancePID;
    PololuVNH3SP30 driver;
    InterruptEncoder encoder;
    double velocitySteering = 0.0f;
    double distanceSteering = 0.0f;
    double inputDistance = 0.0f;
    double inputVelocity = 0.0f;
    double currentVelocity = 0.0f;
    double currentDistance = 0.0f;
    int power = 0;
public:

    PIDDriver (){}
    PIDDriver(PIDvalues vPIDValues, PIDvalues dPIDValues, PololuVNH3SP30Pins driverPins, InterruptEncoderSetup setup){
        velocityPID = PID(vPIDValues);
        distancePID = PID(dPIDValues);
        driver = PololuVNH3SP30(driverPins);
        encoder = InterruptEncoder(setup);
    }

    PIDDriver(PIDvalues vPIDValues, PIDvalues dPIDValues, PololuVNH3SP30Pins driverPins, InterruptEncoderSetup setup, void (encoderInterruptH(bool))){
        velocityPID = PID(vPIDValues);
        distancePID = PID(dPIDValues);
        driver = PololuVNH3SP30(driverPins);
        encoder = InterruptEncoder(setup, encoderInterruptH);
    }

    void inputRelativeDistanceVelocity(double deltaDistance, double velocity){
        inputDistance += deltaDistance;
        inputVelocity = abs(velocity);
    }

    void inputAbsoluteDistanceVelocity(double deltaDistance, double velocity){
        inputDistance = deltaDistance;
        inputVelocity = abs(velocity);
    }

    void handleInterrupt(){
        encoder.handleInterrupt();
    }

    void setResults(double timeDelta){
        currentDistance = encoder.getDistance();
        currentVelocity = encoder.getVelocity(timeDelta);
        distancePID.setResult(currentDistance);
        velocityPID.setResult(currentVelocity);
    }

    void compute(){
        distanceSteering = distancePID.computeNewSteering(LOOP_T, inputDistance);
        velocitySteering = velocityPID.computeNewSteering(LOOP_T, inputVelocity);
        power = (int)(((double)pow(2, PWM_RESOLUTION)) * velocitySteering * distanceSteering);
        driver.drive(power);
        encoder.handleEncoder();
    }

    void reset(double position=0.0f, double velocity=0.0f){
        distancePID.reset();
        velocityPID.reset();
        encoder.reset(-position);
        currentDistance = -position;
        inputDistance = 0.0f;
        inputVelocity = velocity;
    }

    void printDiagnostics(){
        Serial.print("cP/cS/S/cV/V:");
        Serial.print(power);
        Serial.print("/");
        Serial.print(currentDistance);
        Serial.print("/");
        Serial.print(inputDistance);
        Serial.print("/");
        Serial.print(currentVelocity);
        Serial.print("/");
        Serial.print(velocitySteering);
        Serial.print("; V:P/I/D:");
        Serial.print(velocityPID.getP());
        Serial.print("/");
        Serial.print(velocityPID.getI());
        Serial.print("/");
        Serial.print(velocityPID.getD());
        Serial.print("; S:P/I/D:");
        Serial.print(distancePID.getP());
        Serial.print("/");
        Serial.print(distancePID.getI());
        Serial.print("/");
        Serial.print(distancePID.getD());
        Serial.print(" ");
    }

    double getDistance(){
        return currentDistance;
    }
    double getVelocity(){
        return currentVelocity;
    }
    int getPower(){
        return power;
    }
    double getVeloP(){
        return velocityPID.getP();
    }
    double getVeloI(){
        return velocityPID.getI();
    }
    double getVeloD(){
        return velocityPID.getD();
    }
    double getVeloSteering(){
        return velocitySteering;
    }
    double getDistP(){
        return distancePID.getP();
    }
    double getDistI(){
        return distancePID.getI();
    }
    double getDistD(){
        return distancePID.getD();
    }
    double getDistSteering(){
        return distanceSteering;
    }

    void setVPID(PIDConfig pidC){
        if(pidC.velocityPdefined){
            velocityPID.setP(pidC.velocityP);
        }
        if(pidC.velocityIdefined){
            velocityPID.setP(pidC.velocityI);
        }
        if(pidC.velocityDdefined){
            velocityPID.setP(pidC.velocityD);
        }
    }

    void setDPID(PIDConfig pidC){
        if(pidC.velocityPdefined){
            distancePID.setP(pidC.velocityP);
        }
        if(pidC.velocityIdefined){
            distancePID.setP(pidC.velocityI);
        }
        if(pidC.velocityDdefined){
            distancePID.setP(pidC.velocityD);
        }
    }

    PIDConfig getVPID(){
        PIDConfig pidC;
        pidC.velocityP = velocityPID.getP();
        pidC.velocityI = velocityPID.getI();
        pidC.velocityD = velocityPID.getD();
        return pidC;
    }

    PIDConfig getDPID(){
        PIDConfig pidC;
        pidC.velocityP = distancePID.getP();
        pidC.velocityI = distancePID.getI();
        pidC.velocityD = distancePID.getD();
        return pidC;
    }

    void setCallbackFunction(void (interruptFunction(bool))){
        encoder.setCallbackFunction(interruptFunction);
    }
};
#endif
