#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H
#include "Arduino.h"
class Logger{
private:
    static void log(char* level, char* data, bool endl){
        num = (num + 1) % 1000000000;
        Serial.printf("%010lu:%s:%s", num, level, data);
        if(endl) Serial.print("\n");
    }
public:
    static uint32_t num;
    static void error(char* data, bool endl=true){
        log("ERR", data, endl);
    }
    static void warning(char* data, bool endl=true){
        log("WRN", data, endl);
    }
    static void info(char* data, bool endl=true){
        log("INF", data, endl);
    }
    static void debug(char* data, bool endl=true){
        log("DBG", data, endl);
    }

};

uint32_t Logger::num = 0;

#endif