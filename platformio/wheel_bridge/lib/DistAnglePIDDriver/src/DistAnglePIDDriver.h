#ifndef DistAnglePIDDriver_H
#define DistAnglePIDDriver_H
#include "PID.h"
#include "pins.h"
#include "HardwareEncoder.h"
#include "PololuVNH3SP30.h"

struct DistAnglePIDDriverConfig {
    int distPWM, distA; 
    int anglPWM, anglA;
};

struct DistAnglePIDDriverStruct {
    PID velocityPID;
    PID distancePID;
    PololuVNH3SP30 driver;
    double velocitySteering = 0.0f;
    double distanceSteering = 0.0f;
    double inputDistance = 0.0f;
    double inputVelocity = 0.0f;
    double currentVelocity = 0.0f;
    double currentDistance = 0.0f;
    double increment = 0.0f;
    double input = 0.0f;
    int power = 0;
};
class DistAnglePIDDriver {
private:
    PIDvalues distanceVPID =    {DIST_V_P, DIST_V_I, DIST_V_D, MIN_VPID, MAX_PID };
    PIDvalues distanceDPID =    {DIST_D_P, DIST_D_I, DIST_D_D, MIN_DPID, MAX_PID };
    PIDvalues angleVPID =       {ANGL_V_P, ANGL_V_I, ANGL_V_D, MIN_VPID, MAX_PID };
    PIDvalues angleDPID =       {ANGL_D_P, ANGL_D_I, ANGL_D_D, MIN_DPID, MAX_PID };

    HardwareEncoder<1> anglEncoder;
    HardwareEncoder<2> distEncoder;
    
    DistAnglePIDDriverStruct angle;
    DistAnglePIDDriverStruct distance;

    int state = STATE_ANGLE_ENCODER_NOT_INITIALIZED;

    unsigned long requestTicks = 0;

public:
    static const int STATE_ANGLE_ENCODER_NOT_INITIALIZED = 0;
    static const int STATE_READY           = 1;
    static const int STATE_BUSY            = 2;

    DistAnglePIDDriver(){}
    DistAnglePIDDriver(DistAnglePIDDriverConfig config){
        PololuVNH3SP30Pins  distancePins = {config.distPWM, config.distA};
        PololuVNH3SP30Pins  anglePins = {config.anglPWM, config.anglA};
        this->distance.velocityPID = PID(distanceVPID);
        this->distance.distancePID = PID(distanceDPID);
        this->distance.driver = PololuVNH3SP30(distancePins);

        this->angle.velocityPID = PID(angleVPID);
        this->angle.distancePID = PID(angleDPID);
        this->angle.driver = PololuVNH3SP30(anglePins);

        this->anglEncoder.setup(ANGL_ENC2DIST);
        this->distEncoder.setup(DIST_ENC2DIST);
        this->anglEncoder.start();
        this->distEncoder.start();

    }

    void reset(void (*optoInterrupt)()){
        state = STATE_ANGLE_ENCODER_NOT_INITIALIZED;
        #if SLAVE_I2C_ID == 2 || SLAVE_I2C_ID == 4
        digitalWrite(ANGL_P1, LOW);
        #else
        digitalWrite(ANGL_P1, HIGH);
        #endif
        analogWrite(ANGL_PWM, 0.7f*pow(2, PWM_RESOLUTION));
        delay(LOOP_T*30);
        pinMode(ANGL_OPTO, INPUT);
        attachInterrupt(digitalPinToInterrupt(ANGL_OPTO), optoInterrupt, LOW);
    }

    void setResults(double timeDelta){
        this->distance.currentDistance = this->distEncoder.calcPosnF();
        this->distance.currentVelocity = this->distEncoder.calcSpeedF(timeDelta);
        this->distance.distancePID.setResult(this->distance.currentDistance);
        this->distance.velocityPID.setResult(this->distance.currentVelocity);

        this->angle.currentDistance = this->anglEncoder.calcPosnF();
        this->angle.currentVelocity = this->anglEncoder.calcSpeedF(timeDelta);
        this->angle.distancePID.setResult(this->angle.currentDistance);
        this->angle.velocityPID.setResult(this->angle.currentVelocity);
    }

    void compute(){
        if(state == STATE_BUSY){
            if(requestTicks>0){
                this->distance.input += this->distance.increment;
                this->angle.input += this->angle.increment;
                requestTicks--;
            }else{
                state = STATE_READY;
                // this->distance.input = this->distance.currentDistance;
                this->distance.inputVelocity = DIST_IDLE_VELOCITY;

                // this->angle.input = this->angle.currentDistance;
                this->angle.inputVelocity = ANGL_IDLE_VELOCITY;
                // this->angle.distancePID.reset();
                // this->angle.velocityPID.reset();
            }
        }
        this->distance.distanceSteering = this->distance.distancePID.computeNewSteering(LOOP_T, this->distance.input);
        this->distance.velocitySteering = this->distance.velocityPID.computeNewSteering(LOOP_T, this->distance.inputVelocity);
        
        this->angle.distanceSteering = this->angle.distancePID.computeNewSteering(LOOP_T, this->angle.input);
        this->angle.velocitySteering = this->angle.velocityPID.computeNewSteering(LOOP_T, this->angle.inputVelocity);

        this->distance.power = (int)(((double)pow(2, PWM_RESOLUTION)-1) * this->distance.velocitySteering * this->distance.distanceSteering);
        this->angle.power = (int)(((double)pow(2, PWM_RESOLUTION)-1) * this->angle.velocitySteering * this->angle.distanceSteering);
        
        this->distance.driver.drive(this->distance.power);
        this->angle.driver.drive(this->angle.power);
    }

    void inputRelativeDistanceAngleTime(double distance, double angle, unsigned long timeMS){
        requestTicks = timeMS/LOOP_T;

        this->distance.increment = distance/requestTicks;
        this->distance.inputVelocity = 10.0f * abs(distance/(double)requestTicks);

        this->angle.increment = angle/requestTicks;
        this->angle.inputVelocity = 10.0f * abs(angle/(double)requestTicks);

        state = STATE_BUSY;
    }

    void inputAbsoluteDistanceAngleTime(double distance, double angle, unsigned long timeMS){
        double relativeDistance = distance - this->distance.input;
        double relativeAngle = angle - this->angle.input;

        this->inputRelativeDistanceAngleTime(relativeDistance, relativeAngle, timeMS);
    }

    void inputRelativeDistanceTime(double distance, unsigned long timeMS){
        requestTicks = timeMS/LOOP_T;

        this->distance.increment = distance/(double)requestTicks;
        this->distance.inputVelocity = 10.0f * abs(distance/(double)requestTicks);

        state = STATE_BUSY;
    }
    
    void inputAbsoluteDistanceTime(double distance, unsigned long timeMS){
        double relativeDistance = distance - this->distance.input;
        this->inputRelativeDistanceTime(relativeDistance, timeMS);
    }
    
    void inputRelativeAngleTime(double angle, unsigned long timeMS){
        requestTicks = timeMS/LOOP_T;

        this->angle.increment = angle/requestTicks;
        this->angle.inputVelocity = 10.0f * abs(angle/(double)requestTicks);

        state = STATE_BUSY;
    }
    
    void inputAbsoluteAngleTime(double angle, unsigned long timeMS){
        double relativeAngle = angle - this->angle.input;
        this->inputRelativeAngleTime(relativeAngle, timeMS);
    }

    void initialize(double angle=0.0f, unsigned long timeMS=0){
        requestTicks = timeMS/LOOP_T;
        double inputVelocity = 10.0f * abs(angle/(double)requestTicks);

        this->distance.input = 0.0f;
        this->distance.inputVelocity = 0.0f;
        this->distance.currentDistance = 0.0f;
        this->distance.currentVelocity = 0.0f;

        this->angle.input = 0.0f;
        this->angle.inputVelocity = inputVelocity;
        this->angle.currentDistance = 0.0f;
        this->angle.currentVelocity = 0.0f;


        this->distance.distancePID.reset(0.0f);
        this->distance.velocityPID.reset(0.0f);
        this->distEncoder.zeroFTM();

        this->angle.distancePID.reset();
        this->angle.velocityPID.reset();
        this->anglEncoder.zeroFTM();

        state = STATE_BUSY;
    }

    void printDiagnostics(){
        Serial.print(this->distance.input, LOG_ACCURACY);
        Serial.print("/");
        Serial.print(this->distance.currentDistance, LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->distance.inputVelocity, LOG_ACCURACY);
        Serial.print("/");
        Serial.print(this->distance.currentVelocity, LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->distance.power);
        Serial.print("=2^11*");
        Serial.print(this->distance.distanceSteering, LOG_ACCURACY);
        Serial.print("*");
        Serial.print(this->distance.velocitySteering, LOG_ACCURACY);
        Serial.print(" dPID:");
        Serial.print(this->distance.distancePID.getP(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->distance.distancePID.getI(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->distance.distancePID.getD(), LOG_ACCURACY);
        Serial.print(" vPID:");
        Serial.print(this->distance.velocityPID.getP(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->distance.velocityPID.getI(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->distance.velocityPID.getD(), LOG_ACCURACY);
        Serial.print("\t");
        Serial.print(this->angle.input, LOG_ACCURACY);
        Serial.print("/");
        Serial.print(this->angle.currentDistance, LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->angle.inputVelocity, LOG_ACCURACY);
        Serial.print("/");
        Serial.print(this->angle.currentVelocity, LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->angle.power);
        Serial.print("=2^11*");
        Serial.print(this->angle.distanceSteering, LOG_ACCURACY);
        Serial.print("*");
        Serial.print(this->angle.velocitySteering, LOG_ACCURACY);
        Serial.print(" dPID:");
        Serial.print(this->angle.distancePID.getP(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->angle.distancePID.getI(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->angle.distancePID.getD(), LOG_ACCURACY);
        Serial.print(" vPID:");
        Serial.print(this->angle.velocityPID.getP(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->angle.velocityPID.getI(), LOG_ACCURACY);
        Serial.print(":");
        Serial.print(this->angle.velocityPID.getD(), LOG_ACCURACY);
        Serial.print("\n");
    }

    int getState(){
        return state;
    }

    bool checkBusy(){
        return state != STATE_READY;
    }

    double getDistance(){
        return this->distance.currentDistance;
    }

    double getDistanceVelocity(){
        return this->angle.currentVelocity;
    }

    double getDistanceSteering(){
        return this->distance.distanceSteering;
    }

    double getDistanceVelocitySteering(){
        return this->distance.velocitySteering;
    }

    int getDistancePower(){
        return this->distance.power;
    }

    double getAngle(){
        return this->angle.currentDistance;
    }

    double getAngleVelocity(){
        return this->angle.currentVelocity;
    }

    double getAngleSteering(){
        return this->angle.distanceSteering;
    }

    double getAngleVelocitySteering(){
        return this->angle.velocitySteering;
    }

    int getAnglePower(){
        return this->angle.power;
    }
    
};
#endif
