#ifndef CRC_H
#define CRC_H

#include "FastCRC.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

size_t strlen(const char *s) {
    size_t i;
    for (i = 0; s[i] != '\0'; i++) ;
    return i;
}

#define MEM_LEN 256

class CRC{
private:
    FastCRC8 CRC8;
    uint32_t lastCRC = 123;

    uint32_t countCRC(char *data){
        uint16_t dataLen = strlen(data);
        uint8_t *crcData = (uint8_t*)calloc(dataLen+1, sizeof(uint8_t));
        for(int i=0; i<dataLen+1; i++){
            crcData[i] = (uint8_t)data[i];
        }
        uint32_t crc = CRC8.maxim(crcData, dataLen+1);
        return crc;
    }


public:

    uint32_t getLastCRC(){
        return lastCRC;
    }

    void appendCRC(char *&data, char *delimeter){
        char tmpData[MEM_LEN];
        char *tmp = &(tmpData[0]);
        memset(tmp, 0, MEM_LEN);
        sprintf(tmp, "%s:%lu;", data, countCRC(data));
        data = tmp;
    }

    bool checkCRC(char *data, char *delimeter){
        char *tmp = (char*)calloc(strlen(data)+1, sizeof(char));
        strcpy(tmp, data);
        char *last;
        char *token = strtok_r(tmp, delimeter, &last);
        while(true){
            char *tmpToken = strtok_r(NULL, delimeter, &last);
            if(tmpToken == NULL) break;
            token = tmpToken;
        }
        sscanf(token, "%lu", &lastCRC);
        data[strlen(data)-strlen(token)-1] = '\0';
        uint32_t currentCRC = countCRC(data);
        return lastCRC == currentCRC;
    }
    
};
#endif