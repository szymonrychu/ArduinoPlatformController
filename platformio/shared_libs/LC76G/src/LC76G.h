#ifndef _L76X_H_
#define _L76X_H_

#include "DEV_Config.h"
#include <math.h>
#include <stdlib.h>



#define MAXLINELENGTH 120
#define PI            3.14159265358979324
#define A             6378245.0
#define EE            0.00669342162296594323
#define X_PI          PI * 3000.0 / 180.0

//Startup mode
#define HOT_START       "$PMTK101"
#define WARM_START      "$PMTK102"
#define COLD_START      "$PMTK103"
#define FULL_COLD_START "$PMTK104"

//Standby mode -- Exit requires high level trigger
#define SET_PERPETUAL_STANDBY_MODE      "$PMTK161"

#define SET_PERIODIC_MODE               "$PMTK225"
#define SET_NORMAL_MODE                 "$PMTK225,0"
#define SET_PERIODIC_BACKUP_MODE        "$PMTK225,1,1000,2000"
#define SET_PERIODIC_STANDBY_MODE       "$PMTK225,2,1000,2000"
#define SET_PERPETUAL_BACKUP_MODE       "$PMTK225,4"
#define SET_ALWAYSLOCATE_STANDBY_MODE   "$PMTK225,8"
#define SET_ALWAYSLOCATE_BACKUP_MODE    "$PMTK225,9"

//Set the message interval,100ms~10000ms
#define SET_POS_FIX         "$PMTK220"
#define SET_POS_FIX_100MS   "$PMTK220,100"
#define SET_POS_FIX_200MS   "$PMTK220,200"
#define SET_POS_FIX_400MS   "$PMTK220,400"
#define SET_POS_FIX_800MS   "$PMTK220,800"
#define SET_POS_FIX_1S      "$PMTK220,1000"
#define SET_POS_FIX_2S      "$PMTK220,2000"
#define SET_POS_FIX_4S      "$PMTK220,4000"
#define SET_POS_FIX_8S      "$PMTK220,8000"
#define SET_POS_FIX_10S     "$PMTK220,10000"

//Switching time output
#define SET_SYNC_PPS_NMEA_OFF   "$PMTK255,0"
#define SET_SYNC_PPS_NMEA_ON    "$PMTK255,1"

//Baud rate
#define SET_NMEA_BAUDRATE           "$PMTK251"
#define SET_NMEA_BAUDRATE_115200    "$PMTK251,115200"
#define SET_NMEA_BAUDRATE_57600     "$PMTK251,57600"
#define SET_NMEA_BAUDRATE_38400     "$PMTK251,38400"
#define SET_NMEA_BAUDRATE_19200     "$PMTK251,19200"
#define SET_NMEA_BAUDRATE_14400     "$PMTK251,14400"
#define SET_NMEA_BAUDRATE_9600      "$PMTK251,9600"
#define SET_NMEA_BAUDRATE_4800      "$PMTK251,4800"

//To restore the system default setting
#define SET_REDUCTION               "$PMTK314,-1"

//Set NMEA sentence output frequencies 
#define SET_NMEA_OUTPUT "$PMTK314,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0"

char const temp[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

typedef struct {
	double Lon;     //GPS Latitude and longitude
	double Lat;
    char Lon_area;
    char Lat_area;
    UBYTE Time_H;   //Time
    UBYTE Time_M;
    UBYTE Time_S;
    UBYTE Status;   //1:Successful positioning 0：Positioning failed
}GPS;

typedef struct {
    double Lon;
    double Lat;
}Coordinates;

class L76X {

private:
    HardwareSerial *serial;
    uint16_t bufferPointer = 0;
    volatile char buffer1[MAXLINELENGTH] = {0};
    volatile char buffer2[MAXLINELENGTH] = {0};
    volatile char *currentBuffer;
    volatile char *lastBuffer;
    bool received = false;

    double transformLon(double x,double y){
        double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
        ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
        ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
        ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
        return ret;
    }

    double transformLat(double x,double y){
        double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 *sqrt(abs(x));
        ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
        ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
        ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
        return ret;
    }

    Coordinates transform(Coordinates gps){
        Coordinates gg;
        double dLat = transformLat(gps.Lon - 105.0, gps.Lat - 35.0);
        double dLon = transformLon(gps.Lon - 105.0, gps.Lat - 35.0);
        double radLat = gps.Lat / 180.0 * PI;
        double magic = sin(radLat);
        magic = 1 - EE * magic * magic;
        double sqrtMagic = sqrt(magic);
        dLat = (dLat * 180.0) / ((A * (1 - EE)) / (magic * sqrtMagic) * PI);
        dLon = (dLon * 180.0) / (A / sqrtMagic * cos(radLat) * PI);
        gg.Lat = gps.Lat + dLat;
        gg.Lon = gps.Lon + dLon;
        return gg;
    } 

    void sendCommand(char *data){
        char check = data[1], checkChar[3]={0};
        uint8_t i = 0;
        this->serial->print('\r');
        this->serial->print('\n');
        for(i=2; data[i] != '\0'; i++){
            check ^= data[i];
        }
        checkChar[0] = temp[check/16%16];
        checkChar[1] = temp[check%16];
        checkChar[2] = '\0';
        this->serial->print(data);
        this->serial->print('*');
        this->serial->print(checkChar);
        this->serial->print('\r');
        this->serial->print('\n');
    }

public:
    L76X(HardwareSerial *ser){
        currentBuffer = buffer1;
        lastBuffer = buffer2;
        this->serial = ser;
    }
    

    bool begin(uint32_t baud=115200){
        this->serial->begin(baud);
    }

    bool available(){
        return this->serial->available();
    }

    char read(){
        char c = this->serial->read();
        this->currentBuffer[bufferPointer] = c;
        bufferPointer++;
        if(bufferPointer >= MAXLINELENGTH) bufferPointer = MAXLINELENGTH - 1;
        if(c == '\n'){
            currentBuffer[bufferPointer] = 0;
            if(currentBuffer == buffer1){
                currentBuffer = buffer2;
                lastBuffer = buffer1;
            } else {
                currentBuffer = buffer1;
                lastBuffer = buffer2;
            }
            bufferPointer = 0;
            this->received = true;
        }
        return c;
    }

    char newNMAReceived(){
        return this->received;
    }

    char *lastNMA(){
        this->received = false;
        return (char*)lastBuffer;
    }
    
    bool check(char *nmea){
        return true;
    }

    bool parse(char *nmea){
        if (!check(nmea)) return false;

        char *p = nmea;
        p = strchr(p, ',') + 1;

    }


};

void L76X_Send_Command(char *data);
Coordinates L76X_Baidu_Coordinates(void){
    Coordinates temp;
    temp.Lat =((int)(GPS.Lat)) + (GPS.Lat - ((int)(GPS.Lat)))*100 / 60;
    temp.Lon =((int)(GPS.Lon)) + (GPS.Lon - ((int)(GPS.Lon)))*100 / 60;
    temp = transform(temp);
    temp = bd_encrypt(temp);
    return temp;
}
Coordinates L76X_Google_Coordinates(void){
    Coordinates temp;
    GPS.Lat =((int)(GPS.Lat)) + (GPS.Lat - ((int)(GPS.Lat)))*100 / 60;
    GPS.Lon =((int)(GPS.Lon)) + (GPS.Lon - ((int)(GPS.Lon)))*100 / 60;
    temp = transform(temp);
    return temp;
}
GNRMC L76X_Gat_GNRMC(void);
void L76X_Exit_BackupMode(void);

#endif
