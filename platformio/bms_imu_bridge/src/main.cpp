// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_GPS.h>
#include "SerialLogger.h"

#define QUAT_STABILITY_GATE 0.001
#define MIN_STABLE_READINGS_N 100

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// uncomment one combo 9-DoF!
// #include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...
#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
//#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// pick your filter! slower == better quality output
// Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter;  // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

// #define ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
// #define ADAFRUIT_SENSOR_CALIBRATION_USE_SDFAT
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#elif ADAFRUIT_SENSOR_CALIBRATION_USE_SDFAT
  Adafruit_Sensor_Calibration_SDFat cal;
#else
  Adafruit_Sensor_Calibration cal;
  cal.mag_hardiron[3] = {3.65, 24.39, -5.86};
  cal.mag_softiron[9] = {0.98, 0.03, 0.0, 0.03, 0.99, 0.0, 0.0, 0.0, 1.03};
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 1
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;
bool initialPoseStabilized = false;
float pQW, pQX, pQY, pQZ = 0;
float offsets[] = { 0, 0, 0, 0, 0, 0, 0 };
int readingsCount = 0;

bool quaternionIsStable(float qW, float qX, float qY, float qZ){
  bool result = abs(offsets[0] - qW) < QUAT_STABILITY_GATE;
  result = result && abs(offsets[1] - qX) < QUAT_STABILITY_GATE;
  result = result && abs(offsets[2] - qY) < QUAT_STABILITY_GATE;
  result = result && abs(offsets[3] - qZ) < QUAT_STABILITY_GATE;
  if(result){
    readingsCount++;
  }else{
    readingsCount = 0;
  }
  if(readingsCount == 0){
    offsets[0] = qW; offsets[1] = qX; offsets[2] = qY; offsets[3] = qZ;
    offsets[4] = filter.getRoll(); offsets[5] = filter.getPitch(); offsets[6] = filter.getYaw();
  }
  if (readingsCount > MIN_STABLE_READINGS_N) {
    offsets[0] = qW; offsets[1] = qX; offsets[2] = qY; offsets[3] = qZ;
    offsets[4] = filter.getRoll(); offsets[5] = filter.getPitch(); offsets[6] = filter.getYaw();
    return true;
  }
}

void setup() {
  Serial.begin(115200);
  // while (!Serial) yield();

  if (!cal.begin()) {
    // Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    // Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    while (1) delay(10);
  }
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
  delay(1000);
}

void serialEvent1() {
  char c = GPS.read();
  // if (Serial && c) Serial.print(c);
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
}

void loop() {
  float gx, gy, gz;
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
  
  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  if(!initialPoseStabilized) initialPoseStabilized = quaternionIsStable(qw, qx, qy, qz);

  if(Serial){
    Logger::info("", false);
    Serial.print("Q:");
    Serial.print(initialPoseStabilized ? "T" : "F");
    Serial.print(",");
    Serial.print(qw - offsets[0], 4);
    Serial.print(",");
    Serial.print(qx - offsets[1], 4);
    Serial.print(",");
    Serial.print(qy - offsets[2], 4);
    Serial.print(",");
    Serial.print(qz - offsets[3], 4);
    Serial.print(":RPY:");
    Serial.print(filter.getRoll() - offsets[4], 4);
    Serial.print(",");
    Serial.print(filter.getPitch() - offsets[5], 4);
    Serial.print(",");
    Serial.print(filter.getYaw() - offsets[6], 4);
    Serial.print(":GPS:");
    Serial.print(GPS.fix ? "T" : "F");
    Serial.print(",");
    if(GPS.fix){
      Serial.print((int)GPS.fixquality);
      Serial.print(",");
      Serial.print((int)GPS.satellites);
      Serial.print(",");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(",");
      Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
      Serial.print(","); Serial.print(GPS.speed);
      Serial.print(","); Serial.print(GPS.angle);
      Serial.print(","); Serial.print(GPS.altitude);
    }else{
      Serial.print(",,,,,,");
    }
    Serial.println("");
  }
  delay(1);
}