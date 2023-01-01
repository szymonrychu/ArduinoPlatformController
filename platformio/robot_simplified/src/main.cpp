#include "Arduino.h"
// #include "pins.h"
#include "Command.h"
#include "SerialLogger.h"
#include "RobotSimplified.h"
#include "EveryTimer.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

#define MEM_LEN 256

long loopCounter = 0;
unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;
char buffer[MEM_LEN];

void printDiagnostics();

EveryTimer busyDiagnosticsTimer(0.01, printDiagnostics);
EveryTimer freeDiagnosticsTimer(0.25, printDiagnostics);

Command command = Command(" ", "\n");
RobotSimplified robot;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void serialEvent() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        command.inputChar(ch);
    }
}


void printDiagnostics(){
    sensors_event_t angVelocityData , linearAccelData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion quat = bno.getQuat();
    char* data = robot.getDiagnostics();
    sprintf(buffer, ":%.4f:%.4f:%.4f:%.4f:%.4f:%.4f:%.4f:%.4f:%.4f:%.4f",
            quat.w(), quat.x(), quat.y(), quat.z(),
            angVelocityData.gyro.x, angVelocityData.gyro.y, angVelocityData.gyro.z,
            linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z);
    Logger::info(data); //, false);
    // Serial.println(buffer);
}

void defaultFunc(char* data){
    Logger::error(data);
}

void G10HandleLeftRightSteering(){
    /*
G10 10 10 8
G10 -10 -10 8
G10 10 -10 8
G10 24 24 20
G10 -24 -24 20
G10 2 0 1.5
G10 0 1 2
G10 0.1 0.1 3
G10 -0.1 -0.1 1

G10 0 -0.1 5
G10 0 0.1 5

G10 -0.1 0 5
G10 0.1 0 5

    */
    // 
    MoveRequest m;
    if(m.parseRequest(command)){
        char*data = robot.requestMove(m);
        Logger::info(data);
    }else{
        Logger::error("Error parsing!");
    }
}


void setup(){
    memset(buffer, 0, MEM_LEN);
    Serial.begin(115200);
    while(Serial){;} // Wait for someone to open Serial port
    command.addDefaultHandler(defaultFunc);
    command.addCommand("G10", G10HandleLeftRightSteering);
    
    while(!bno.begin())  {
        delay(1000);
    }
    bno.setExtCrystalUse(true);
    Wire.setClock(400000); // 400KHz
}

void loop(){
    robot.compute();
    command.parse();
    if(robot.isBusy()){
        busyDiagnosticsTimer.compute();
    }else {
        freeDiagnosticsTimer.compute();
    }
}
