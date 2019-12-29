#include "Arduino.h"
#include "DistAnglePIDDriver.h"
#include "i2c_t3.h"
#include <queue>
#include "WireCommunication.h"

#define MEM_LEN 256
#define STATUS_MS 250

bool calibrated = false;
bool received = false;

DistAnglePIDDriverConfig config = {
    DIST_PWM, DIST_P1, DIST_ENC1, DIST_ENC2, \
    ANGL_PWM, ANGL_P1, ANGL_ENC1, ANGL_ENC2, \
    NULL, NULL
};

DistAnglePIDDriver driver = DistAnglePIDDriver(config);
SlaveWire slaveWire = SlaveWire(":", ";");

void distanceInterrupt(){
    driver.handleDistanceInterrupt();
}

void angleInterrupt(){
    driver.handleAngleInterrupt();
}

void optoInterrupt(){
    bool prevState = digitalRead(ANGL_OPTO) != LOW;
    delay(2);
    if(!calibrated && digitalRead(ANGL_OPTO) != HIGH && prevState){
        calibrated = true;
        digitalWrite(ANGL_P1, LOW);
        analogWrite(ANGL_PWM, 0);
        detachInterrupt(digitalPinToInterrupt(ANGL_OPTO));
        driver.reset();
    }
}
/* 
G00FuncHandler - add distance value to what's already been requested
  relative from the end of last requested position
*/
void G00FuncHandler(){
    double distance = 0.0f; char* distanceCH = slaveWire.next();
    double distanceVelocity = 0.0f; char* distanceVelocityCH = slaveWire.next();
    distance = atof(distanceCH);
    distanceVelocity = atof(distanceVelocityCH);
    #if SLAVE_I2C_ID == 1 || SLAVE_I2C_ID == 4
    driver.inputRelativeDistanceVelocity(-distance, distanceVelocity);
    #else
    driver.inputRelativeDistanceVelocity(distance, distanceVelocity);
    #endif
    slaveWire.reportSuccess("G00:OK;\0");
}

/* 
G01FuncHandler - add angle value to what's already been requested
  relative from the end of last requested position
*/
void G01FuncHandler(){
    double angle = 0.0f; char* angleCH = slaveWire.next();
    double angleVelocity = 0.0f; char* angleVelocityCH = slaveWire.next();
    angle = atof(angleCH);
    angleVelocity = atof(angleVelocityCH);
    driver.inputRelativeAngleVelocity(angle, angleVelocity);
    slaveWire.reportSuccess("G01:OK;\0");
}

/* 
G02FuncHandler - add distance value to what's been currently reached
*/
void G02FuncHandler(){
    double distance = 0.0f; char* distanceCH = slaveWire.next();
    double distanceVelocity = 0.0f; char* distanceVelocityCH = slaveWire.next();
    distance = atof(distanceCH);
    distanceVelocity = atof(distanceVelocityCH);
    #if SLAVE_I2C_ID == 1 || SLAVE_I2C_ID == 4
    driver.inputAbsoluteDistanceVelocity(driver.getDistance()-distance, distanceVelocity);
    #else
    driver.inputAbsoluteDistanceVelocity(driver.getDistance()+distance, distanceVelocity);
    #endif
    slaveWire.reportSuccess("G02:OK;\0");
}

/* 
G03FuncHandler - add angle value to what's been currently reached
*/
void G03FuncHandler(){
    double angle = 0.0f; char* angleCH = slaveWire.next();
    double angleVelocity = 0.0f; char* angleVelocityCH = slaveWire.next();
    angle = atof(angleCH);
    angleVelocity = atof(angleVelocityCH);
    driver.inputAbsoluteAngleVelocity(angle + driver.getAngle(), angleVelocity);
    slaveWire.reportSuccess("G03:OK;\0");
}

PIDConfig commandParserParsePID(){
    PIDConfig pidC;
    char*dPch = slaveWire.next();
    char*dIch = slaveWire.next();
    char*dDch = slaveWire.next();
    char*vPch = slaveWire.next();
    char*vIch = slaveWire.next();
    char*vDch = slaveWire.next();
    
    if(dPch != NULL && strcmp("-", dPch) != 0) {
        pidC.distanceP = atof(dPch);
        pidC.distancePdefined = true;
    }
    if(dIch != NULL && strcmp("-", dIch) != 0) {
        pidC.distanceI = atof(dIch);
        pidC.distanceIdefined = true;
    }
    if(dDch != NULL && strcmp("-", dDch) != 0) {
        pidC.distanceD = atof(dDch);
        pidC.distanceDdefined = true;
    }
    if(vPch != NULL && strcmp("-", vPch) != 0) {
        pidC.velocityP = atof(vPch);
        pidC.velocityPdefined = true;
    }
    if(vIch != NULL && strcmp("-", vIch) != 0) {
        pidC.velocityI = atof(vIch);
        pidC.velocityIdefined = true;
    }
    if(vDch != NULL && strcmp("-", vDch) != 0) {
        pidC.velocityD = atof(vDch);
        pidC.velocityDdefined = true;
    }
    return pidC;
}

/* 
G10FuncHandler - set distance PID
*/
void G10FuncHandler(){
    PIDConfig pidC = commandParserParsePID();
    driver.setDistancePID(pidC);
    slaveWire.reportSuccess("G10:OK;\0");
}

/* 
G11FuncHandler - set distance velocity PID
*/
void G11FuncHandler(){
    PIDConfig pidC = commandParserParsePID();
    driver.setDistanceVelocityPID(pidC);
    slaveWire.reportSuccess("G11:OK;\0");
}

/* 
G12FuncHandler - set angle PID
*/
void G12FuncHandler(){
    PIDConfig pidC = commandParserParsePID();
    driver.setAnglePID(pidC);
    slaveWire.reportSuccess("G12:OK;\0");
}

/* 
G13FuncHandler - set angle velocity PID
*/
void G13FuncHandler(){
    PIDConfig pidC = commandParserParsePID();
    driver.setAngleVelocityPID(pidC);
    slaveWire.reportSuccess("G13:OK;\0");
}

void resetAnglePosition(){
    calibrated = false;
    #if SLAVE_I2C_ID == 2 || SLAVE_I2C_ID == 4
    digitalWrite(ANGL_P1, LOW);
    #else
    digitalWrite(ANGL_P1, HIGH);
    #endif
    analogWrite(ANGL_PWM, 0.7f*pow(2, PWM_RESOLUTION));
    delay(LOOP_T*30);
    pinOptoSetup(optoInterrupt);
    driver.reset();
    double angleVelocity = 0.7;
    #if SLAVE_I2C_ID == 2 || SLAVE_I2C_ID == 4
    double angle = -10.0;
    #else
    double angle = 10.0;
    #endif
    driver.inputAbsoluteAngleVelocity(angle + driver.getAngle(), angleVelocity + driver.getAngleVelocity());
}

/*
G99FuncHandler - resets driver
*/
void G99FuncHandler(){
    resetAnglePosition();
    slaveWire.reportSuccess("G99:OK;\0");
}

// void captureStatus(){
//     RawStatusStructure status;
//     unsigned long currentTime = millis();
//     status.timestamp = currentTime - time;
//     time = currentTime;
//     //TODO add more data to RawStatusStructure
//     while(operationOnQueuePending) delayMicroseconds(10);
//     operationOnQueuePending = true;
//     statusQueue.push(status);
//     if(statusQueue.size() > maxStatusVectorSize){
//         statusQueue.pop();
//     }
//     operationOnQueuePending = false;
// }

// void G98FuncHandler(){
//     while(operationOnQueuePending) delayMicroseconds(10);
//     operationOnQueuePending = true;
//     if(!statusQueue.empty()){
//         RawStatusStructure tmpStatus = statusQueue.front();
//         int remaingSize = statusQueue.size();
//         //TODO handle more data from RawStatusStructure
//         sprintf(outputDataBuffer, "%lu:%d", tmpStatus.timestamp, remaingSize);
//         newDataToTransmit = true;
//         statusQueue.pop();
//     }else{
//         strcpy(outputDataBuffer, "0:0");
//         newDataToTransmit = true;
//     }
//     operationOnQueuePending = false;
// }



void requestEvent(){
    slaveWire.onRequestEvent();
}

void receiveEvent(size_t count){
    slaveWire.onReceiveEvent(count);
}

void defaultFunc(char* data){
    slaveWire.reportProblem(data);
}

void setup(){
    Wire.begin(I2C_SLAVE, SLAVE_I2C_ID, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    Wire.setDefaultTimeout(50000);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    slaveWire.addDefaultHandler(defaultFunc);
    slaveWire.addCommand("G00", G00FuncHandler);
    slaveWire.addCommand("G01", G01FuncHandler);
    slaveWire.addCommand("G02", G02FuncHandler);
    slaveWire.addCommand("G03", G03FuncHandler);

    // pid handling functions 
    slaveWire.addCommand("G10", G10FuncHandler);
    slaveWire.addCommand("G11", G11FuncHandler);
    slaveWire.addCommand("G12", G12FuncHandler);
    slaveWire.addCommand("G13", G13FuncHandler);

    // slaveWire.addCommand("G98", G98FuncHandler);
    slaveWire.addCommand("G99", G99FuncHandler);
    pinsSetup(distanceInterrupt, angleInterrupt);
    resetAnglePosition();
}

void loop(){
    slaveWire.parse();
    if(calibrated)driver.compute();
    delay(LOOP_T);
    if(calibrated)driver.setResults(LOOP_T);
}


