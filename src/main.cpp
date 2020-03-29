#include "Arduino.h"
#include "DistAnglePIDDriver.h"
#include <queue>
#include "Command.h"
#include "pins.h"
#include "SerialLogger.h"
uint32_t Logger::num = 0;

#define MEM_LEN 256
#define STATUS_MS 250

bool calibrated = false;
bool received = false;
char* lastCommand = "";

DistAnglePIDDriverConfig config = {
    DIST_PWM, DIST_P1, DIST_ENC1, DIST_ENC2, \
    ANGL_PWM, ANGL_P1, ANGL_ENC1, ANGL_ENC2, \
    NULL, NULL
};

DistAnglePIDDriver driver = DistAnglePIDDriver(config);
Command command = Command(" ", "\n");

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
        digitalWrite(ANGL_P1, LOW);
        analogWrite(ANGL_PWM, 0);
        detachInterrupt(digitalPinToInterrupt(ANGL_OPTO));
        driver.reset();
        Logger::info("Calibrated:", false);
        Serial.print(lastCommand);
        Serial.print('\n');
        calibrated = true;
    }
}
/* 
G00FuncHandler - add distance value to what's already been requested
  relative from the end of last requested position
*/
void G00FuncHandler(){
    double distance = 0.0f; char* distanceCH = command.next();
    double distanceVelocity = 0.0f; char* distanceVelocityCH = command.next();
    distance = atof(distanceCH);
    distanceVelocity = atof(distanceVelocityCH);
    #if SLAVE_I2C_ID == 1 || SLAVE_I2C_ID == 4
    driver.inputAbsoluteDistanceVelocity(-distance, distanceVelocity);
    #else
    driver.inputAbsoluteDistanceVelocity(distance, distanceVelocity);
    #endif
    lastCommand = "G00";
    Logger::info("G00 ACK");
}

/* 
G01FuncHandler - add angle value to what's already been requested
  relative from the end of last requested position
*/
void G01FuncHandler(){
    double angle = 0.0f; char* angleCH = command.next();
    double angleVelocity = 0.0f; char* angleVelocityCH = command.next();
    angle = atof(angleCH);
    angleVelocity = atof(angleVelocityCH);
    driver.inputAbsoluteAngleVelocity(angle, angleVelocity);
    lastCommand = "G01";
    Logger::info("G01 ACK");
}

/* 
G02FuncHandler - add distance value to what's been currently reached
*/
void G02FuncHandler(){
    double distance = 0.0f; char* distanceCH = command.next();
    double distanceVelocity = 0.0f; char* distanceVelocityCH = command.next();
    distance = atof(distanceCH);
    distanceVelocity = atof(distanceVelocityCH);
    #if SLAVE_I2C_ID == 1 || SLAVE_I2C_ID == 4
    driver.inputAbsoluteDistanceVelocity(driver.getDistance()-distance, distanceVelocity);
    #else
    driver.inputAbsoluteDistanceVelocity(driver.getDistance()+distance, distanceVelocity);
    #endif
    lastCommand = "G02";
    Logger::info("G02 ACK");
}

/* 
G03FuncHandler - add angle value to what's been currently reached
*/
void G03FuncHandler(){
    double angle = 0.0f; char* angleCH = command.next();
    double angleVelocity = 0.0f; char* angleVelocityCH = command.next();
    angle = atof(angleCH);
    angleVelocity = atof(angleVelocityCH);
    driver.inputAbsoluteAngleVelocity(angle + driver.getAngle(), angleVelocity);
    lastCommand = "G03";
    Logger::info("G03 ACK");
}

PIDConfig commandParserParsePID(){
    PIDConfig pidC;
    char*dPch = command.next();
    char*dIch = command.next();
    char*dDch = command.next();
    char*vPch = command.next();
    char*vIch = command.next();
    char*vDch = command.next();
    
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
    lastCommand = "G10";
    Logger::info("G10 ACK");
}

/* 
G11FuncHandler - set distance velocity PID
*/
void G11FuncHandler(){
    PIDConfig pidC = commandParserParsePID();
    driver.setDistanceVelocityPID(pidC);
    lastCommand = "G11";
    Logger::info("G11 ACK");
}

/* 
G12FuncHandler - set angle PID
*/
void G12FuncHandler(){
    PIDConfig pidC = commandParserParsePID();
    driver.setAnglePID(pidC);
    lastCommand = "G12";
    Logger::info("G12 ACK");
}

/* 
G13FuncHandler - set angle velocity PID
*/
void G13FuncHandler(){
    PIDConfig pidC = commandParserParsePID();
    driver.setAngleVelocityPID(pidC);
    lastCommand = "G13";
    Logger::info("G13 ACK");
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
}

/*
G99FuncHandler - resets driver
*/
void G99FuncHandler(){
    resetAnglePosition();
    lastCommand = "G99";
    Logger::info("G99 ACK");
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

void G98FuncHandler(){
    driver.printDiagnostics();
}

void serialEvent() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        command.inputChar(ch);
    }
    digitalWrite(LED_BUILTIN, HIGH);
}

void defaultFunc(char* data){
    Logger::error(data);
}

void angleReachedHandler(bool pass){
    Logger::info("a_reached:", false);
    Serial.print(lastCommand);
    Serial.print(":");
    Serial.print(driver.getAngle());
    Serial.print('\n');
}

void distanceReachedHandler(bool pass){
    Logger::info("d_reached:", false);
    Serial.print(lastCommand);
    Serial.print(":");
    Serial.print(driver.getDistance());
    Serial.print('\n');
}

void setup(){
    Serial.begin(115200);

    command.addDefaultHandler(defaultFunc);
    command.addCommand("G00", G00FuncHandler);
    command.addCommand("G01", G01FuncHandler); // G03 10 0.5 
    command.addCommand("G02", G02FuncHandler);
    command.addCommand("G03", G03FuncHandler);

    // pid handling functions 
    command.addCommand("G10", G10FuncHandler);
    command.addCommand("G11", G11FuncHandler);
    command.addCommand("G12", G12FuncHandler);
    command.addCommand("G13", G13FuncHandler);

    command.addCommand("G98", G98FuncHandler);
    command.addCommand("G99", G99FuncHandler); // G99
    driver.setAngleCallbackFunction(angleReachedHandler);
    driver.setDistanceCallbackFunction(distanceReachedHandler);
    pinsSetup(distanceInterrupt, angleInterrupt);
    resetAnglePosition();
}

int loopCounter = 0;
void loop(){
    command.parse();
    if(calibrated)driver.compute();
    delay(LOOP_T);
    if(calibrated)driver.setResults(LOOP_T);
    if(loopCounter == 0){
        driver.printDiagnostics();
    }
    loopCounter = (loopCounter+1)%100;
}

// G03 10 0.7
