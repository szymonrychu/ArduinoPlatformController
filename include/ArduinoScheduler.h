#include "pins.h"

#define MEM_LEN 256
#define SERIAL_DETAILED_STATUS_ERROR "WireError"
#define SERIAL_RECEIVED_GARBAGE "SerialReceivedGarbage"
#define SERIAL_DETAILED_TRANSMIT_OK "SerialTransmitOK"

#include "WireCommunication.h"
#include "Command.h"
#include "i2c_t3.h"
#include "SerialLogger.h"
uint32_t Logger::num = 0;

Command command = Command(" ", "\n");
MasterWire masterWire = MasterWire(":", ";");

char databuf[MEM_LEN];


bool i2cERROR = false;

/* Handling incoming bytes from Serial */
void serialEvent() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        command.inputChar(ch);
    }
}

/* Triggered on i2c bus error */
void errorEvent(void){ 
    switch(Wire.status()) {
        case I2C_TIMEOUT:  Serial.printf("%s:I2C timeout\n", SERIAL_DETAILED_STATUS_ERROR); i2cERROR = true; Wire.resetBus(); break;
        case I2C_ADDR_NAK: Serial.printf("%s:Slave addr not acknowledged\n", SERIAL_DETAILED_STATUS_ERROR); i2cERROR = true; break;
        case I2C_DATA_NAK: Serial.printf("%s:Slave data not acknowledged\n", SERIAL_DETAILED_STATUS_ERROR); i2cERROR = true; break;
        case I2C_ARB_LOST: Serial.printf("%s:Arbitration Lost, possible pullup problem\n", SERIAL_DETAILED_STATUS_ERROR); i2cERROR = true; Wire.resetBus(); break;
        case I2C_BUF_OVF:  Serial.printf("%s:I2C buffer overflow\n", SERIAL_DETAILED_STATUS_ERROR); i2cERROR = true; break;
        case I2C_NOT_ACQ:  Serial.printf("%s:Cannot acquire bus, possible stuck SDA/SCL\n", SERIAL_DETAILED_STATUS_ERROR); i2cERROR = true; Wire.resetBus(); break;
        case I2C_DMA_ERR:  Serial.printf("%s:DMA Error\n", SERIAL_DETAILED_STATUS_ERROR); i2cERROR = true; break;
        default:           break;
    }
}


void G001transmitTo0x01(){ // G004 G03:10:10
    Logger::info(masterWire.request(0x01, command.next(), true));
}

void G002transmitTo0x02(){
    Logger::info(masterWire.request(0x02, command.next(), true));
}

void G003transmitTo0x03(){
    Logger::info(masterWire.request(0x03, command.next(), true));
}

void G004transmitTo0x04(){
    Logger::info(masterWire.request(0x04, command.next(), true));
}

void G100multicastToAll(){
    char *message = command.next();
    for(int addr=0; addr<4; addr++){
        Logger::info(masterWire.request(addr+1, message, true));
    }
}


/*
    defaultFunc()
    handle unrecognized command by echoing all data sent over
    through Serial while also parsing it out
*/
void defaultFunc(char* data){
    Serial.printf("%s:%s\n", SERIAL_RECEIVED_GARBAGE, data);
}

/* Reset bus upon error  */
void G999resetBus(){
    Wire.resetBus();
    // Wire.finish();
    i2cERROR = false; 
}



/* Sending to Serial once transmission to i2c slave was succesful */
void transmitDone(void){ Serial.printf("%s\n", SERIAL_DETAILED_TRANSMIT_OK); }

void setup(){
    memset(databuf, 0, sizeof(databuf));
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
    Wire.setDefaultTimeout(50000);
    Wire.onError(errorEvent);
    Serial.begin(115200);
    command.addDefaultHandler(defaultFunc);
    // commandParser.addCommand("G000", G000getStatus); // G000 1\n; G000 4
    command.addCommand("G001", G001transmitTo0x01); // G001 GXX:YY:ZZ\n G001 G00:00:00 G001 G00:01:0.05 G001 G01:01:0.05
    command.addCommand("G002", G002transmitTo0x02); // G002 GXX:YY:ZZ\n G002 G00:00:00
    command.addCommand("G003", G003transmitTo0x03); // G003 GXX:YY:ZZ\n G003 G00:00:00 
    command.addCommand("G004", G004transmitTo0x04); // G004 GXX:YY:ZZ\n G004 G01:00:00
    command.addCommand("G100", G100multicastToAll); // G100 GXX:YY:ZZ\n G100 G00:00:00 G100 G03:50:0.5
    command.addCommand("G999", G999resetBus); // G999 (space at the end)
    delay(1000);
    // postReset();
}

void loop(){
    command.parse();
    delay(LOOP_T);
}

//G004 G0asddsa00:00
//G004 G03:20:0.5
//G004 G04:20:0.5