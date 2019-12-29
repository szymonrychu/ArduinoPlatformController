#ifndef WIRE_COMMUNICATION_H
#define WIRE_COMMUNICATION_H
#include "CRC.h"
#include "i2c_t3.h"
#include "Command.h"

#define MEM_LENGTH 256
#define EMPTY_RESPONSE "Null\0"
#define EMPTY_RESPONSE_SIZE 5
#define REQUEST_ACK "ACK"
#define RESPONSE_SUCCESS "OK\0"
#define RESPONSE_FAILURE "NOK\0"

class MasterWire: public CRC{
private:
    char* delim;
    char* term;
    char receiveBuffer[MEM_LENGTH];
public:
    MasterWire(char *delimeter, char *terminator){
		delim = delimeter;
		term = terminator;
	}
    void transmit(uint8_t address, char* data, bool checksum=false){
        if(checksum)appendCRC(data, delim);
        Wire.beginTransmission(address);
        Wire.write(data, strlen(data)+1);
        Wire.endTransmission();
    }

    char* request(uint8_t address, char* what=REQUEST_ACK, bool checksum=false){
        if(checksum)appendCRC(what, delim);
        Wire.beginTransmission(address);
        Wire.write(what, strlen(what)+1);
        Wire.endTransmission(I2C_NOSTOP);
        bool responseIsNull = true;
        char *result;
        for(int c=0;c<10 && responseIsNull; c++){
            Wire.requestFrom(address, MEM_LENGTH, I2C_STOP);
            int d=0;
            for(; d<MEM_LENGTH-1 && Wire.available(); d++){
                receiveBuffer[d] = Wire.readByte();
                receiveBuffer[d+1] = '\0';
            }
            if(d>0){
                result = (char*)calloc(strlen(receiveBuffer)+1, sizeof(char));
                strcpy(result, receiveBuffer);
                responseIsNull = strcmp(result, EMPTY_RESPONSE) == 0;
            }else{
                delay(1);
            }
        }
        return result;
    }
};

class SlaveWire: public Command, CRC{
private:
    bool integrityHandler(char *data){
        return checkCRC(data, delim);
    }

protected:
    char transmitBuffer[MEM_LENGTH];
    uint16_t transmitBufferLength = 0;
    char receiveBuffer[MEM_LENGTH];
    uint16_t receiveBufferLength = 0;

public:
    SlaveWire(char *delimeter, char *terminator) : Command(delimeter, terminator){
        memset(transmitBuffer, '\0', MEM_LENGTH);
        memset(receiveBuffer, '\0', MEM_LENGTH);
	}

    void onReceiveEvent(size_t count){
        char tmp[MEM_LENGTH];
        memset(tmp, '\0', MEM_LENGTH);
        Wire.read(tmp, Wire.available());
        inputString(tmp);
    }

    void onRequestEvent(){
        if(transmitBufferLength > 0){
            Wire.write(transmitBuffer, transmitBufferLength);
            transmitBufferLength = 0;
        }else{
            Wire.write(EMPTY_RESPONSE, EMPTY_RESPONSE_SIZE);
        }
    }

    bool appendDataToSendOutFromSlaveToMaster(char *data){
        transmitBufferLength = strlen(data);
        strcpy(transmitBuffer, data);

        // int inputDataLength = strlen(data)+1;
        // if(transmitBufferLength + inputDataLength > MEM_LENGTH - 2) return false;
        // strcpy(transmitBuffer + transmitBufferLength, data);
        // transmitBufferLength+=inputDataLength;
        // *(transmitBuffer + transmitBufferLength) = delim[0];
        // *(transmitBuffer + transmitBufferLength + 1) = '\0';
        return true;
    }

    void reportSuccess(char* data=NULL){
        if(data){
            appendDataToSendOutFromSlaveToMaster(data);
        }else{
            appendDataToSendOutFromSlaveToMaster(RESPONSE_SUCCESS);
        }
    }

    void reportProblem(char* data=NULL, char* what=NULL){
        if(data){
            appendDataToSendOutFromSlaveToMaster(data);
        }else{
            appendDataToSendOutFromSlaveToMaster(RESPONSE_SUCCESS);
        }
    }
};


#endif