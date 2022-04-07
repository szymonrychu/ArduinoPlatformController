#include "Arduino.h"
// #include "pins.h"
#include "Command.h"
#include "SerialLogger.h"
#include "SmartWheel.h"
#include "EveryTimer.h"

#define MEM_LEN 256

long loopCounter = 0;
unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;
char buffer[MEM_LEN];

void printDiagnostics();

// EveryTimer diagnosticsTimer(0.01, printDiagnostics);
Command command = Command(" ", "\n");
SmartWheel smartWheel;

void serialEvent() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        command.inputChar(ch);
    }
}


void printDiagnostics(){
    char* data = smartWheel.getDiagnostics();
    Logger::info(data);
}

void defaultFunc(char* data){
    Logger::error(data);
}

void G10HandleDistanceAngleTime(){
    /*
    Rotate 30deg in 1.5s:
G10 0 0.52359877 1.5
    Rotate 45deg in 2s:
G10 0 0.78539816 4
    Rotate 90deg in 4s:
G10 0 1.57079632 4
    Rotate 180deg in 8s:
G10 0 3.14159265 8
    Rotate 360deg in 5s:
G10 0 6.28318530 6
G10 0 0 6
    Move forward 2m in 10s:
G10 2 0 10
    Move forward 2m in 20s:
G10 2 0 20
G10 -2 0 20

    */
    // 
    MoveRequest m;
    if(m.parseRequest(command)){
        char*data = smartWheel.requestMove(m);
        Logger::info(data);
    }else{
        Logger::error("Error parsing!");
    }
}

void G90HandleReset(){
    /*
    Reset wheel to 0 angle:
G90
    */
    Logger::info("Reset");
    smartWheel.requestReset();
}

void setup(){
    memset(buffer, 0, MEM_LEN);
    Serial.begin(115200);
    while(Serial){;} // Wait for someone to open Serial port
    command.addDefaultHandler(defaultFunc);
    command.addCommand("G10", G10HandleDistanceAngleTime);
    command.addCommand("G90", G90HandleReset);
    
}

void loop(){
    smartWheel.compute();
    command.parse();
    if(smartWheel.isBusy()){
        printDiagnostics();
    //     diagnosticsTimer.compute();
    }
}
