#include "Arduino.h"
#include "DistAnglePIDDriver.h"
#include <queue>
#include "Command.h"
#include "pins.h"
#include "SerialLogger.h"
uint32_t Logger::num = 0;

#define MEM_LEN 256
#define STATUS_MS 250

static const int STATE_FRESH      = 0;
static const int STATE_READY      = 1;
static const int STATE_ANGLE_INIT = 2;
static const int STATE_PROCESSING = 4;

int state = STATE_FRESH;
char* getStateString(){
    char stateStr[] = {
        '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'
    };
    char *stateStrPTR = &(stateStr[0]);
    switch(state){
        case STATE_FRESH:
            stateStrPTR = "FRESH";
            return stateStrPTR;
        case STATE_READY:
            stateStrPTR = "READY";
            return stateStrPTR;
        case STATE_ANGLE_INIT:
            stateStrPTR = "ANGLE_INIT";
            return stateStrPTR;
        case STATE_PROCESSING:
            stateStrPTR = "PROCESSING";
            return stateStrPTR;
    }
}

char* lastCommand = "";

DistAnglePIDDriverConfig config = {
    DIST_PWM, DIST_P1, \
    ANGL_PWM, ANGL_P1
};

DistAnglePIDDriver driver = DistAnglePIDDriver(config);
Command command = Command(" ", "\n");

void optoInterrupt(){

    bool prevState = digitalRead(ANGL_OPTO) != LOW;
    delay(2);
    if(digitalRead(ANGL_OPTO) != HIGH && prevState){
        digitalWrite(ANGL_P1, LOW);
        analogWrite(ANGL_PWM, 0);
        detachInterrupt(digitalPinToInterrupt(ANGL_OPTO));
        state = STATE_ANGLE_INIT;
    }
}

/* 
G00FuncHandler - add distance value to what's already been requested
  relative from the end of last requested position
*/
void G00FuncHandler(){
    double distance = 0.0f; char* distanceCH = command.next();
    unsigned long timeMS = 0; char* timeMSCH = command.next();
    distance = atof(distanceCH);
    timeMS = atol(timeMSCH);
    #if SLAVE_I2C_ID == 1 || SLAVE_I2C_ID == 4
    driver.inputAbsoluteDistanceTime(-distance, timeMS);
    #else
    driver.inputAbsoluteDistanceTime(distance, timeMS);
    #endif
    lastCommand = "G00";
    Logger::info("ACK:G00");
}

/* 
G01FuncHandler - add angle value to what's already been requested
  relative from the end of last requested position
*/
void G01FuncHandler(){
    double angle = 0.0f; char* angleCH = command.next();
    unsigned long timeMS = 0; char* timeMSCH = command.next();
    angle = atof(angleCH);
    timeMS = atol(timeMSCH);
    driver.inputAbsoluteAngleTime(angle, timeMS);
    lastCommand = "G01";
    Logger::info("ACK:G01");
}

/* 
G02FuncHandler - add distance value to what's been currently reached
G02 50.0 300
*/
void G02FuncHandler(){
    double distance = 0.0f; char* distanceCH = command.next();
    unsigned long timeMS = 0; char* timeMSCH = command.next();
    distance = atof(distanceCH);
    timeMS = atol(timeMSCH);
    driver.inputRelativeDistanceTime(distance, timeMS);
    lastCommand = "G02";
    Logger::info("ACK:G02");
}

/* 
G03FuncHandler - add angle value to what's been currently reached
G03 10.0 200
G03 -10.0 200
*/
void G03FuncHandler(){
    double angle = 0.0f; char* angleCH = command.next();
    unsigned long timeMS = 0; char* timeMSCH = command.next();
    angle = atof(angleCH);
    timeMS = atol(timeMSCH);
    driver.inputRelativeAngleTime(angle, timeMS);
    lastCommand = "G03";
    Logger::info("ACK:G03");
}

/* 
G09FuncHandler - relative distance and absolute angle
*/
void G09FuncHandler(){
    double distance = 0.0f; char* distanceCH = command.next();
    double angle = 0.0f; char* angleCH = command.next();
    unsigned long timeMS = 0; char* timeMSCH = command.next();
    angle = atof(angleCH);
    distance = atof(distanceCH);
    timeMS = atol(timeMSCH);
    driver.inputRelativeDistanceAngleTime(distance, angle, timeMS);
    lastCommand = "G09";
    Logger::info("ACK:G09");
}

/*
G99FuncHandler - resets driver
*/
void G99FuncHandler(){
    state = STATE_FRESH;
    driver.reset(optoInterrupt);
    lastCommand = "G99";
    Logger::info("ACK:G99");
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
    command.addCommand("G09", G09FuncHandler);

    command.addCommand("G98", G98FuncHandler);
    command.addCommand("G99", G99FuncHandler); // G99
    driver.reset(optoInterrupt);
}
/*
G09 -50.0 -50.0 1000
G09 50.0 50.0 1000
G09 10.0 20.0 200
G09 10.0 30.0 200

G09 0.0 -50.0 1000
G09 0.0 50.0 1000

G09 -50.0 0.0 1000
G09 50.0 0.0 1000

G09 0.0 -50.0 400
G09 0.0 50.0 400

G09 0.0 -50.0 500
G09 0.0 50.0 500
*/

short counter = 0;
void loop(){
    if(state != STATE_READY){
        Logger::info(getStateString(), false);
        Serial.print(":");
        driver.printDiagnostics();
    }else if(counter == 0){
        Logger::info(getStateString(), false);
        Serial.print(":");
        driver.printDiagnostics();
    }
    command.parse();
    switch(state){
        case STATE_READY:
        case STATE_PROCESSING:
            driver.compute();
            delay(LOOP_T);
            driver.setResults(LOOP_T);
            if(driver.checkBusy()){
                state = STATE_PROCESSING;
            }else{
                state = STATE_READY;
            }
            break;
        case STATE_ANGLE_INIT:
            // #if SLAVE_I2C_ID == 2 || SLAVE_I2C_ID == 4
            // digitalWrite(ANGL_P1, LOW);
            // #else
            // digitalWrite(ANGL_P1, HIGH);
            // #endif
            // analogWrite(ANGL_PWM, 0.7f*pow(2, PWM_RESOLUTION));
            // delay(LOOP_T*30);
            // analogWrite(ANGL_PWM, 0);
            #if SLAVE_I2C_ID == 2 || SLAVE_I2C_ID == 4
            double angle = -10.0;
            #else
            double angle = 10.0;
            #endif
            driver.initialize(angle, 100);
            state = STATE_READY;
            break;
    }
    counter = (counter+1)%100;
}