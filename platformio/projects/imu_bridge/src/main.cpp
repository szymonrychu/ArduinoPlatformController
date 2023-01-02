#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

void setup(void){
  Serial.begin(115200);
  while(!Serial){delay(1000);} // Wait for someone to open Serial port

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  while(!bno.begin())  {
    Serial.println("INIT_ERROR");
    delay(1000);
  }
  bno.setExtCrystalUse(true);
  Wire.setClock(400000); // 400KHz
}

void serialEvent1() {
  char c = GPS.read();
  // if (Serial && c) Serial.print(c);
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
}

void loop(void){
  imu::Quaternion quat = bno.getQuat();
  int8_t temp = bno.getTemp();

  Serial.print(quat.w(), 4);
  Serial.print(",");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.print(quat.z(), 4);
  Serial.print(",");
  Serial.print(temp);
  if(GPS.fix){
    int gps_DD_latitude = int(GPS.latitude/100.0);
    float gps_MMMM_latitude = GPS.latitude - 100.0*gps_DD_latitude;
    float gps_decimal_latitude = gps_DD_latitude + float(gps_MMMM_latitude)/60.0;

    int gps_DDD_longitude = int(GPS.longitude/100.0);
    float gps_MMMM_longitude = GPS.longitude - 100.0*gps_DDD_longitude;
    float gps_decimal_longitude = gps_DDD_longitude + float(gps_MMMM_longitude)/60.0;

    Serial.print((int)GPS.fixquality);
    Serial.print(",");
    Serial.print((int)GPS.satellites);
    Serial.print(",");
    Serial.print(gps_decimal_latitude, 6);
    // Serial.print(GPS.lat);
    Serial.print(",");
    Serial.print(gps_decimal_longitude, 6);
    // Serial.print(GPS.lon);
    Serial.print(","); Serial.print(GPS.speed);
    Serial.print(","); Serial.print(GPS.angle);
    Serial.print(","); Serial.print(GPS.altitude);
  }else{
    Serial.print(",,,,,,");
  }
  Serial.println("");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
