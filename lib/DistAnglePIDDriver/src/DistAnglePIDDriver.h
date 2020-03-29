#ifndef DistAnglePIDDriver_H
#define DistAnglePIDDriver_H
#include "PIDDriver.h"

struct DistAnglePIDDriverConfig {
    int distPWM, distA, distENC1, distENC2; 
    int anglPWM, anglA, anglENC1, anglENC2;
    void (*distEncoderFunction)(bool) = NULL;
    void (*anglEncoderFunction)(bool) = NULL;
};
class DistAnglePIDDriver {
private:
    PIDvalues distanceVPID =    {DIST_V_P, DIST_V_I, DIST_V_D, MIN_VPID, MAX_PID };
    PIDvalues distanceDPID =    {DIST_D_P, DIST_D_I, DIST_D_D, MIN_DPID, MAX_PID };
    PIDvalues angleVPID =       {ANGL_V_P, ANGL_V_I, ANGL_V_D, MIN_VPID, MAX_PID };
    PIDvalues angleDPID =       {ANGL_D_P, ANGL_D_I, ANGL_D_D, MIN_DPID, MAX_PID };

    PIDDriver distDriver;
    PIDDriver anglDriver;

    double inputDistance = 0.0f;
    double inputDistanceVelocity = 0.0f;
    double inputAngle = 0.0f;
    double inputAngleVelocity = 0.0f;

public:
    DistAnglePIDDriver(){}
    DistAnglePIDDriver(DistAnglePIDDriverConfig config){
        PololuVNH3SP30Pins  distancePins = {config.distPWM, config.distA};
        PololuVNH3SP30Pins  anglePins = {config.anglPWM, config.anglA};
        InterruptEncoderSetup distanceEncoderSetup = {config.distENC1, config.distENC2, DIST_ENC2DIST};
        InterruptEncoderSetup angleEncoderSetup = {config.anglENC1, config.anglENC2, ANGL_ENC2DIST};

        distDriver = PIDDriver(distanceVPID, distanceDPID, distancePins, distanceEncoderSetup, config.distEncoderFunction);
        anglDriver = PIDDriver(angleVPID, angleDPID, anglePins, angleEncoderSetup, config.anglEncoderFunction);

    }

    void setResults(double timeDelta){
        distDriver.setResults(timeDelta);
        anglDriver.setResults(timeDelta);
    }

    void compute(){
        distDriver.compute();
        anglDriver.compute();
    }
    
    void handleDistanceInterrupt(){
        distDriver.handleInterrupt();
    }
    
    void handleAngleInterrupt(){
        anglDriver.handleInterrupt();
    }

    void inputRelativeDistanceAngleTime(double distance, double angle, long timeMS){
        double distanceSpeed = distance / ((double)(timeMS)/1000.0f);
        double angleSpeed = angle / ((double)(timeMS)/1000.0f);
        distDriver.inputAbsoluteDistanceVelocity(distance, distanceSpeed);
        anglDriver.inputAbsoluteDistanceVelocity(angle, angleSpeed);
    }

    void inputAbsoluteDistanceVelocity(double distance, double velocity){
        distDriver.inputAbsoluteDistanceVelocity(distance, velocity);
    }

    void inputRelativeDistanceVelocity(double distance, double velocity){
        distDriver.inputRelativeDistanceVelocity(distance, velocity);
    }
    
    void inputAbsoluteAngleVelocity(double angle, double velocity){
        anglDriver.inputAbsoluteDistanceVelocity(angle, velocity);
    }
    
    void inputRelativeAngleVelocity(double angle, double velocity){
        anglDriver.inputRelativeDistanceVelocity(angle, velocity);
    }

    void reset(){

        double angleVelocity = 0.7;
        #if SLAVE_I2C_ID == 2 || SLAVE_I2C_ID == 4
        double angle = -10.0;
        #else
        double angle = 10.0;
        #endif

        inputDistance = 0.0f;
        inputDistanceVelocity = 0.0f;
        distDriver.reset();
        inputAngle = 0.0f;
        inputAngleVelocity = 0.0f;
        anglDriver.reset(angle, angleVelocity);
    }

    void printDiagnostics(){
        Serial.print("dist:");
        distDriver.printDiagnostics();
        Serial.print("angl:");
        anglDriver.printDiagnostics();
        Serial.print("\n");
    }

    double getDistance(){
        return distDriver.getDistance();
    }

    double getDistanceVelocity(){
        return distDriver.getVelocity();
    }

    double getDistanceSteering(){
        return distDriver.getDistSteering();
    }

    double getDistanceVelocitySteering(){
        return distDriver.getVeloSteering();
    }

    int getDistancePower(){
        return distDriver.getPower();
    }

    double getAngle(){
        return anglDriver.getDistance();
    }

    double getAngleVelocity(){
        return anglDriver.getVelocity();
    }

    double getAngleSteering(){
        return anglDriver.getDistSteering();
    }

    double getAngleVelocitySteering(){
        return anglDriver.getVeloSteering();
    }

    int getAnglePower(){
        return anglDriver.getPower();
    }

    PIDConfig getAnglePID(){
        return anglDriver.getDPID();
    }

    PIDConfig getAngleVelocityPID(){
        return anglDriver.getVPID();
    }

    PIDConfig getDistancePID(){
        return distDriver.getDPID();
    }

    PIDConfig getDistanceVelocityPID(){
        return distDriver.getVPID();
    }

    void setAnglePID(PIDConfig pidC){
        anglDriver.setDPID(pidC);
    }

    void setAngleVelocityPID(PIDConfig pidC){
        anglDriver.setVPID(pidC);
    }

    void setDistancePID(PIDConfig pidC){
        distDriver.setDPID(pidC);
    }

    void setDistanceVelocityPID(PIDConfig pidC){
        distDriver.setVPID(pidC);
    }

    void setAngleCallbackFunction(void (interruptFunction(bool))){
        anglDriver.setCallbackFunction(interruptFunction);
    }

    void setDistanceCallbackFunction(void (interruptFunction(bool))){
        distDriver.setCallbackFunction(interruptFunction);
    }
    
};
#endif
