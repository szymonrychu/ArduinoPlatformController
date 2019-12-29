#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H
#include "Arduino.h"
class Logger{
private:
    static void log(char* level, char* data){
        Serial.printf("%lu:%s:%s;\n", num++, level, data);
    }
public:
    static uint32_t num;
    static void error(char* data){
        log("ERR", data);
    }
    static void warning(char* data){
        log("WRN", data);
    }
    static void info(char* data){
        log("INF", data);
    }
    static void debug(char* data){
        log("DBG", data);
    }

};
#endif