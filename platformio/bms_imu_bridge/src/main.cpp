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

#define CHARGING_ENABLE 2
#define POWER_ENABLE 3
#define SYSTEM_ENABLE 15

#define CHARGING_ADC A3
#define BATTERY_ADC A2

#define CHG_FULL_INDICATOR 6

#define BATTERY_MAX_VOLTAGE 4.2*3
#define BATTERY_MIN_VOLTAGE 3.1*3
#define BATTERY_MID_VOLTAGE (BATTERY_MAX_VOLTAGE+BATTERY_MIN_VOLTAGE)/2

#define CHARGING_MIN_VOLTAGE 15

#define MEAN_POINTS_NUM 100
float batteryVoltages[MEAN_POINTS_NUM] = {};
float chargingVoltages[MEAN_POINTS_NUM] = {};
int voltagesPointer = 0;
bool histerezis = true;

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

void setup() {
  pinMode(CHARGING_ENABLE, OUTPUT);
  digitalWrite(CHARGING_ENABLE, HIGH);
  pinMode(POWER_ENABLE, OUTPUT);
  digitalWrite(POWER_ENABLE, HIGH);
  pinMode(SYSTEM_ENABLE, OUTPUT);
  digitalWrite(SYSTEM_ENABLE, HIGH);
  pinMode(CHG_FULL_INDICATOR, INPUT_PULLDOWN);

  Serial.begin(115200);
  for(int c=0; c<MEAN_POINTS_NUM;c++){
    batteryVoltages[c] = 0.0;
    chargingVoltages[c] = 0.0;
  }
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

  if (GPS.newNMEAreceived()) {
  // a tricky thing here is if we print the NMEA sentence, or data
  // we end up not listening and catching other sentences!
  // so be very wary if using OUTPUT_ALLDATA and trying to print out data
  // Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
  if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

void loop() {
  float gx, gy, gz;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
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

  bool systemON = false;
  bool powerON = false;
  bool chargingON = false;
  bool batteryFull = digitalRead(CHG_FULL_INDICATOR);
  float batteryRawVoltage = 12.25*float(analogRead(BATTERY_ADC))/488.0;
  float chargingRawVoltage = 20.5*float(analogRead(CHARGING_ADC))/807.0;
  batteryVoltages[voltagesPointer] = batteryRawVoltage;
  chargingVoltages[voltagesPointer] = chargingRawVoltage;
  voltagesPointer = (voltagesPointer + 1) % MEAN_POINTS_NUM;
  float batteryVoltageSum = 0.0f;
  float chargingVoltageSum = 0.0f;
  for(int c=0; c<MEAN_POINTS_NUM;c++){
    batteryVoltageSum += batteryVoltages[c];
    chargingVoltageSum += chargingVoltages[c];
  }
  float batteryVoltage = batteryVoltageSum / float(MEAN_POINTS_NUM);
  float chargingVoltage = chargingVoltageSum / float(MEAN_POINTS_NUM);

  if((!batteryFull || batteryVoltage<BATTERY_MID_VOLTAGE) && chargingVoltage>CHARGING_MIN_VOLTAGE ){
    chargingON = true;
  }
  if(batteryVoltage < BATTERY_MIN_VOLTAGE){
    histerezis = false;
  }
  if(batteryVoltage > BATTERY_MID_VOLTAGE){
    histerezis = true;
  }
  if(histerezis){
    systemON = true;
    powerON = true;
  }
  digitalWrite(CHARGING_ENABLE, chargingON ? LOW : HIGH);
  digitalWrite(POWER_ENABLE, powerON ? LOW : HIGH);
  digitalWrite(SYSTEM_ENABLE, systemON ? HIGH : LOW);

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  if(Serial){
    Serial.print(qw, 4);
    Serial.print(",");
    Serial.print(qx, 4);
    Serial.print(",");
    Serial.print(qy, 4);
    Serial.print(",");
    Serial.print(qz, 4);
    Serial.print(":");
    Serial.print(batteryFull ? "T" : "F");
    Serial.print(",");
    Serial.print(systemON ? "T" : "F");
    Serial.print(",");
    Serial.print(powerON ? "T" : "F");
    Serial.print(",");
    Serial.print(chargingON ? "T" : "F");
    Serial.print(":");
    Serial.print(batteryVoltage, 2);
    Serial.print(",");
    Serial.print(chargingVoltage, 2);
  }
  if(Serial){
    Serial.print(":");
    Serial.print(GPS.fix ? "T" : "F");
    Serial.print(",");
    Serial.print((int)GPS.fixquality);
    Serial.print(",");
    Serial.print((int)GPS.satellites);
  }
  if (Serial && GPS.fix) {
    Serial.print(",");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(",");
    Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
    Serial.print(","); Serial.print(GPS.speed);
    Serial.print(","); Serial.print(GPS.angle);
    Serial.print(","); Serial.print(GPS.altitude);
  }
  if(Serial)Serial.println("");
  
  
}