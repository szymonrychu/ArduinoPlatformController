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

EveryTimer diagnosticsTimer(0.1, printDiagnostics);
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
    Rotate 60deg in 1.5s:
G10 0 0.5235987756 1.5
    Rotate 90deg in 2s:
G10 0 0.7853981634 2
    Rotate -90deg in 2s:
G10 0 -0.7853981634 4
    Rotate 180deg in 4s:
G10 0 1.5707963268 4
    Move forward 2m in 10s:
G10 2 0 10
    Move forward 2m in 20s:
G10 2 0 20

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
    // if(smartWheel.isBusy()){
        diagnosticsTimer.compute();
    // }
}
