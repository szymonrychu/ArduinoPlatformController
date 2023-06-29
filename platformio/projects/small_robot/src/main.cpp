#include "Arduino.h"
#include "ArduinoJson.h"
#include "pins.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <Servo.h>
#include "LED.h"
#include "ServoWheel.h"
#include "JSONCommand.h"
#include <ArduinoQueue.h>

/*
{"reset":{"queue":true}}

{"turn": {"angle": 0.7853, "velocity": 0.5}}
{"turn": {"angle": -0.7853, "velocity": 0.5}}
{"turn": {"angle": 0.7853, "velocity": 0.5, "x": 0.4}}
{"turn": {"angle": -0.7853, "velocity": 0.5, "x": -0.4}}
{"turn": {"angle": 0.7853, "velocity": 0.5, "x": 0.3, "y": 0.4}}
{"turn": {"angle": -0.7853, "velocity": 0.5, "x": -0.3, "y": 0.4}}
{"turn": {"angle": 0.7853, "velocity": 0.5, "x": 0.3, "y": -0.4}}
{"turn": {"angle": -0.7853, "velocity": 0.5, "x": -0.3, "y": -0.4}}
{"turn": {"angle": 1.5706, "velocity": 0.5}}
{"turn": {"angle": -1.5706, "velocity": 0.5}}
{"turn": {"angle": 3.1412, "velocity": 0.75}}
{"turn": {"angle": -3.1412, "velocity": 0.5}}


{"motor1":{"distance":0.2,"velocity":0.5},"motor2":{"distance":0.2,"velocity":0.5},"motor3":{"distance":0.2,"velocity":0.5},"motor4":{"distance":0.2,"velocity":0.5}}
{"motor1":{"distance":-0.2,"velocity":0.5},"motor2":{"distance":-0.2,"velocity":0.5},"motor3":{"distance":-0.2,"velocity":0.5},"motor4":{"distance":-0.2,"velocity":0.5}}
{"motor1":{"distance":0,"velocity":0},"motor2":{"distance":0,"velocity":0},"motor3":{"distance":0,"velocity":0},"motor4":{"distance":0,"velocity":0}}
{"motor1":{"distance":1.0,"velocity":0.5},"motor2":{"distance":1.0,"velocity":0.5},"motor3":{"distance":1.0,"velocity":0.5},"motor4":{"distance":1.0,"velocity":0.5}}

{"motor1":{"distance":1.0,"velocity":0.5}}
{"motor2":{"distance":1.0,"velocity":0.5}}
{"motor3":{"distance":1.0,"velocity":0.5}}
{"motor4":{"distance":1.0,"velocity":0.5}}

{"motor1":{"angle":0.7853},"motor2":{"angle":0.7853},"motor3":{"angle":0.7853},"motor4":{"angle":0.7853}}
{"motor1":{"angle":-0.7853},"motor2":{"angle":-0.7853},"motor3":{"angle":-0.7853},"motor4":{"angle":-0.7853}}
{"motor1":{"angle":1.5706},"motor2":{"angle":1.5706},"motor3":{"angle":1.5706},"motor4":{"angle":1.5706}}
{"motor1":{"angle":0},"motor2":{"angle":0},"motor3":{"angle":0},"motor4":{"angle":0}}

{"pan":{"angle":0.7853},"tilt":{"angle":0.7853}}
{"pan":{"angle":0},"tilt":{"angle":0}}

{"motor1":{"distance":0.2,"velocity":0.5,"angle":0.7853},"motor2":{"distance":0.2,"velocity":0.5,"angle":0.7853},"motor3":{"distance":0.2,"velocity":0.5,"angle":0.7853},"motor4":{"distance":0.2,"velocity":0.5,"angle":0.7853},{"pan":{"angle":0.7853},"tilt":{"angle":0.7853}}}

*/

#define QUEUE_SIZE_ITEMS 10

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define MEM_LEN 256
char buffer[MEM_LEN];
long loopCounter = 0;
double batteryVoltage = 0;

ArduinoQueue<Move> moveQueue(QUEUE_SIZE_ITEMS);

ServoWheel motor1(1, M1_ENCA, M1_ENCB, M1_HA, M1_HB, M1_SERV);
ServoWheel motor2(2, M2_ENCA, M2_ENCB, M2_HA, M2_HB, M2_SERV);
ServoWheel motor3(3, M3_ENCA, M3_ENCB, M3_HA, M3_HB, M3_SERV);
ServoWheel motor4(4, M4_ENCA, M4_ENCB, M4_HA, M4_HB, M4_SERV);


Servo servoPan;
Servo servoTilt;
Led redLed = Led(USER_LED_1);
Led greenLed = Led(USER_LED_2, true);

Move zeroAngles(){
  MotorMove m;
  m.angle = 0;
  m.angleSet = true;
  Move move;
  move.motor1 = m;
  move.motor2 = m;
  move.motor3 = m;
  move.motor4 = m;
  return move;
}

Move filterAngles(Move m){
  Move r;
  r.motor1.angleSet = m.motor1.angleSet;
  r.motor2.angleSet = m.motor2.angleSet;
  r.motor3.angleSet = m.motor3.angleSet;
  r.motor4.angleSet = m.motor4.angleSet;
  r.motor1.angle = m.motor1.angle;
  r.motor2.angle = m.motor2.angle;
  r.motor3.angle = m.motor3.angle;
  r.motor4.angle = m.motor4.angle;
  return r;
}

Move filterDistanceVelocity(Move m){
  Move r;
  r.motor1.distanceSet = m.motor1.distanceSet;
  r.motor2.distanceSet = m.motor2.distanceSet;
  r.motor3.distanceSet = m.motor3.distanceSet;
  r.motor4.distanceSet = m.motor4.distanceSet;
  r.motor1.distanceDelta = m.motor1.distanceDelta;
  r.motor2.distanceDelta = m.motor2.distanceDelta;
  r.motor3.distanceDelta = m.motor3.distanceDelta;
  r.motor4.distanceDelta = m.motor4.distanceDelta;
  r.motor1.velocity = m.motor1.velocity;
  r.motor2.velocity = m.motor2.velocity;
  r.motor3.velocity = m.motor3.velocity;
  r.motor4.velocity = m.motor4.velocity;
  return r;
}

double limitAngle(double angle){
  if(angle > HALF_PI) return angle - PI;
  if(angle < -HALF_PI) return angle + PI;
  return angle;
}

Move turn(double turnAngleRad, double velocity, double X=0, double Y=0){
  Move m;
  m.motor1.angleSet = true;
  m.motor1.angle = limitAngle(M1_IN_PLACE_TURN * atan2(M1_Y-Y, M1_X-X));
  m.motor2.angleSet = true;
  m.motor2.angle = limitAngle(M2_IN_PLACE_TURN * atan2(M2_Y-Y, M2_X-X));
  m.motor3.angleSet = true;
  m.motor3.angle = limitAngle(M3_IN_PLACE_TURN * atan2(M3_Y-Y, M3_X-X));
  m.motor4.angleSet = true;
  m.motor4.angle = limitAngle(M4_IN_PLACE_TURN * atan2(M4_Y-Y, M4_X-X));

  double m1Radius = sqrt((M1_X-X)*(M1_X-X) + (M1_Y-Y)*(M1_Y-Y));
  double m2Radius = sqrt((M2_X-X)*(M2_X-X) + (M2_Y-Y)*(M2_Y-Y));
  double m3Radius = sqrt((M3_X-X)*(M3_X-X) + (M3_Y-Y)*(M3_Y-Y));
  double m4Radius = sqrt((M4_X-X)*(M4_X-X) + (M4_Y-Y)*(M4_Y-Y));
  // double maxRadius = max(max(m1Radius, m2Radius), max(m3Radius, m4Radius));

  double meanRadius = (m1Radius+m2Radius+m3Radius+m4Radius)/4.0;
  double m1coef = max(m1Radius/meanRadius, MIN_TURN_COEF);
  if(X < M1_X) m1coef = -m1coef;
  double m2coef = max(m2Radius/meanRadius, MIN_TURN_COEF);
  if(X > M2_X) m2coef = -m2coef;
  double m3coef = max(m3Radius/meanRadius, MIN_TURN_COEF);
  if(X > M3_X) m3coef = -m3coef;
  double m4coef = max(m4Radius/meanRadius, MIN_TURN_COEF);
  if(X < M4_X) m4coef = -m4coef;

  m.motor1.distanceSet = true;
  m.motor1.distanceDelta = (1.1 * turnAngleRad * PI * m1Radius)/PI;
  if(X < M1_X) m.motor1.distanceDelta = -m.motor1.distanceDelta;
  m.motor1.velocity = abs(m1coef * velocity);

  m.motor2.distanceSet = true;
  m.motor2.distanceDelta = -(1.1 * turnAngleRad * PI * m2Radius)/PI;
  if(X > M2_X) m.motor2.distanceDelta = -m.motor2.distanceDelta;
  m.motor2.velocity = abs(m2coef * velocity);

  m.motor3.distanceSet = true;
  m.motor3.distanceDelta = -(1.1 * turnAngleRad * PI * m3Radius)/PI;
  if(X > M3_X) m.motor3.distanceDelta = -m.motor3.distanceDelta;
  m.motor3.velocity = abs(m3coef * velocity);

  m.motor4.distanceSet = true;
  m.motor4.distanceDelta = (1.1 * turnAngleRad * PI * m4Radius)/PI;
  if(X < M4_X) m.motor4.distanceDelta = -m.motor4.distanceDelta;
  m.motor4.velocity = abs(m4coef * velocity);
  return m;
}

void commandHandler(char* input){
  DynamicJsonDocument doc(512);
  deserializeJson(doc, input);
  JsonObject obj = doc.as<JsonObject>();
  if(obj.containsKey("reset")){
    if(obj["reset"].containsKey("queue") && obj["reset"]["queue"]){
      while(!moveQueue.isEmpty()){
        moveQueue.dequeue();
      }
    }
  }
  if(obj.containsKey("turn")){
    double x = 0;
    double y = 0;
    if(obj["turn"].containsKey("x")) x = obj["turn"]["x"];
    if(obj["turn"].containsKey("y")) y = obj["turn"]["y"];
    Move move = turn(obj["turn"]["angle"], obj["turn"]["velocity"], x, y);
    moveQueue.enqueue(filterAngles(move));
    moveQueue.enqueue(filterDistanceVelocity(move));
    moveQueue.enqueue(zeroAngles());
    return;
  }

  if(moveQueue.isFull()){
    StaticJsonDocument<128> out;
    out["micros"] = micros();
    out["status"] = "queue_full";
    serializeJson(out, Serial);
    Serial.println("");
  }
  StaticJsonDocument<128> out;

  bool panAngleSet = obj.containsKey("pan") && obj["pan"].containsKey("angle");
  bool tiltAngleSet = obj.containsKey("tilt") && obj["tilt"].containsKey("angle");
  bool motor1AngleSet = obj.containsKey("motor1") && obj["motor1"].containsKey("angle");
  bool motor2AngleSet = obj.containsKey("motor2") && obj["motor2"].containsKey("angle");
  bool motor3AngleSet = obj.containsKey("motor3") && obj["motor3"].containsKey("angle");
  bool motor4AngleSet = obj.containsKey("motor4") && obj["motor4"].containsKey("angle");
  bool motor1DistanceVelocitySet = obj.containsKey("motor1") && obj["motor1"].containsKey("distance") && obj["motor1"].containsKey("velocity");
  bool motor2DistanceVelocitySet = obj.containsKey("motor2") && obj["motor2"].containsKey("distance") && obj["motor2"].containsKey("velocity");
  bool motor3DistanceVelocitySet = obj.containsKey("motor3") && obj["motor3"].containsKey("distance") && obj["motor3"].containsKey("velocity");
  bool motor4DistanceVelocitySet = obj.containsKey("motor4") && obj["motor4"].containsKey("distance") && obj["motor4"].containsKey("velocity");

  Move move;
  move.pan.angleSet = panAngleSet;
  move.pan.angle = obj["pan"]["angle"];

  move.tilt.angleSet = tiltAngleSet;
  move.tilt.angle = obj["tilt"]["angle"];

  move.motor1.angleSet = motor1AngleSet;
  move.motor1.angle = obj["motor1"]["angle"];
  move.motor2.angleSet = motor2AngleSet;
  move.motor2.angle = obj["motor2"]["angle"];
  move.motor3.angleSet = motor3AngleSet;
  move.motor3.angle = obj["motor3"]["angle"];
  move.motor4.angleSet = motor4AngleSet;
  move.motor4.angle = obj["motor4"]["angle"];

  move.motor1.distanceSet = motor1DistanceVelocitySet;
  move.motor1.distanceDelta = obj["motor1"]["distance"];
  move.motor1.velocity = obj["motor1"]["velocity"];
  
  move.motor2.distanceSet = motor2DistanceVelocitySet;
  move.motor2.distanceDelta = obj["motor2"]["distance"];
  move.motor2.velocity = obj["motor2"]["velocity"];
  
  move.motor3.distanceSet = motor3DistanceVelocitySet;
  move.motor3.distanceDelta = obj["motor3"]["distance"];
  move.motor3.velocity = obj["motor3"]["velocity"];
  
  move.motor4.distanceSet = motor4DistanceVelocitySet;
  move.motor4.distanceDelta = obj["motor4"]["distance"];
  move.motor4.velocity = obj["motor4"]["velocity"];

  moveQueue.enqueue(move);

  out["micros"] = micros();
  out["queue_l"] = moveQueue.item_count();
  out["status"] = "OK";
  serializeJson(out, Serial);
  Serial.println("");
}

bool motorsReady(){
  return motor1.ready() && motor2.ready() && motor3.ready() && motor4.ready();
}

JSONCommand command = JSONCommand("\n", commandHandler);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#define GPSSerial Serial4
Adafruit_GPS GPS(&GPSSerial);

void serialEvent() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    command.inputChar(ch);
  }
}

void setup(void){
  Serial.begin(115200);

  while(!Serial){delay(10);} 

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  while(!bno.begin())  {
    Serial.println("INIT_ERROR");
    delay(1000);
  }
  bno.setExtCrystalUse(true);
  Wire.setClock(400000); // 400KHz

  pinMode(USER_BUTTON_1, INPUT);
  pinMode(USER_BUTTON_2, INPUT);
  pinMode(BATT_MEAS_AIN, INPUT);

  servoPan.attach(SERV_PAN);
  servoTilt.attach(SERV_TILT);

  motor1.setup();
  motor2.setup();
  motor3.setup();
  motor4.setup();
}

void serialEvent4() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
}

bool motorDirection = true;
bool servoDirection = true;
uint32_t loopCount = 0;
bool lastMReady = true;

void loop(void){
  sensors_event_t angVelocityData , accData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  int8_t temp = bno.getTemp();

  uint8_t gpsFixQuality = 0;
  uint8_t gpsSatellites = 0;
  float gpsDecimalLatitude = 0.0;
  float gpsDecimalLongitude = 0.0;
  float gpsSpeed = 0.0;
  float gpsAngle = 0.0;
  float gpsAltitude = 0.0;


  if(GPS.fix){
    gpsFixQuality = GPS.fixquality;
    gpsSatellites = GPS.satellites;
    gpsSpeed = GPS.speed;
    gpsAngle = GPS.angle;
    gpsAltitude = GPS.altitude;

    int gps_DD_latitude = int(GPS.latitude/100.0);
    float gps_MMMM_latitude = GPS.latitude - 100.0*gps_DD_latitude;
    gpsDecimalLatitude = gps_DD_latitude + float(gps_MMMM_latitude)/60.0;

    int gps_DDD_longitude = int(GPS.longitude/100.0);
    float gps_MMMM_longitude = GPS.longitude - 100.0*gps_DDD_longitude;
    gpsDecimalLongitude = gps_DDD_longitude + float(gps_MMMM_longitude)/60.0;
  }

  batteryVoltage = 10.82*((float)analogRead(BATT_MEAS_AIN))/498.0;

  bool mReady = motorsReady();
  if(mReady){
    if(!moveQueue.isEmpty()){
      Move m = moveQueue.dequeue();

      motor1.drive(m.motor1);
      motor2.drive(m.motor2);
      motor3.drive(m.motor3);
      motor4.drive(m.motor4);
    }else{
      if(!motor1.isFresh()){
        motor1.resetDistanceVelocity();
      }
      if(!motor2.isFresh()){
        motor2.resetDistanceVelocity();
      }
      if(!motor3.isFresh()){
        motor3.resetDistanceVelocity();
      }
      if(!motor4.isFresh()){
        motor4.resetDistanceVelocity();
      }
    }
  }

  if(loopCount == 0 || !mReady){
    StaticJsonDocument<1536> doc;
    doc["micros"] = micros();
    if(mReady){
      doc["status"] = "ready";
    }else{
      doc["status"] = "busy";
    }
    doc["queue_l"] = moveQueue.item_count();
    doc["int_temp"] = temp;
    JsonObject battery = doc.createNestedObject("battery");
    battery["voltage"] = batteryVoltage;
    JsonObject imu = doc.createNestedObject("imu");
    JsonObject quaternion = imu.createNestedObject("quaternion");
    quaternion["w"] = quat.w();
    quaternion["x"] = quat.x();
    quaternion["y"] = quat.y();
    quaternion["z"] = quat.z();
    JsonObject gyroscope = imu.createNestedObject("gyroscope");
    gyroscope["x"] = angVelocityData.gyro.x;
    gyroscope["y"] = angVelocityData.gyro.y;
    gyroscope["z"] = angVelocityData.gyro.z;
    JsonObject accelerometer = imu.createNestedObject("accelerometer");
    accelerometer["x"] = angVelocityData.acceleration.x;
    accelerometer["y"] = angVelocityData.acceleration.y;
    accelerometer["z"] = angVelocityData.acceleration.z;
    JsonObject gps = doc.createNestedObject("gps");
    gps["fix_quality"] = gpsFixQuality;
    gps["satellites"] = gpsSatellites;
    gps["dec_latitude"] = gpsDecimalLatitude;
    gps["dec_longitude"] = gpsDecimalLongitude;
    gps["speed"] = gpsSpeed;
    gps["angle"] = gpsAngle;
    gps["altitude"] = gpsAltitude;
    JsonObject motor1JSON = doc.createNestedObject("motor1");
    motor1JSON["ready"] = motor1.ready();
    motor1JSON["servo"] = motor1.readServo();
    motor1JSON["distance"] = motor1.currentDistance();
    motor1JSON["distance_error"] = motor1.currentDistanceError();
    motor1JSON["distance_steering"] = motor1.currentDistanceSteering();
    motor1JSON["velocity"] = motor1.currentVelocity();
    motor1JSON["velocity_error"] = motor1.currentDistanceError();
    motor1JSON["velocity_steering"] = motor1.currentVelocityError();
    motor1JSON["steering"] = motor1.currentSteering();
    JsonObject motor2JSON = doc.createNestedObject("motor2");
    motor2JSON["ready"] = motor2.ready();
    motor2JSON["servo"] = motor2.readServo();
    motor2JSON["distance"] = motor2.currentDistance();
    motor2JSON["distance_error"] = motor2.currentDistanceError();
    motor2JSON["distance_steering"] = motor2.currentDistanceSteering();
    motor2JSON["velocity"] = motor2.currentVelocity();
    motor2JSON["velocity_error"] = motor2.currentDistanceError();
    motor2JSON["velocity_steering"] = motor2.currentVelocityError();
    motor2JSON["steering"] = motor2.currentSteering();
    JsonObject motor3JSON = doc.createNestedObject("motor3");
    motor3JSON["ready"] = motor3.ready();
    motor3JSON["servo"] = motor3.readServo();
    motor3JSON["distance"] = motor3.currentDistance();
    motor3JSON["distance_error"] = motor3.currentDistanceError();
    motor3JSON["distance_steering"] = motor3.currentDistanceSteering();
    motor3JSON["velocity"] = motor3.currentVelocity();
    motor3JSON["velocity_error"] = motor3.currentDistanceError();
    motor3JSON["velocity_steering"] = motor3.currentVelocityError();
    motor3JSON["steering"] = motor3.currentSteering();
    JsonObject motor4JSON = doc.createNestedObject("motor4");
    motor4JSON["ready"] = motor4.ready();
    motor4JSON["servo"] = motor4.readServo();
    motor4JSON["distance"] = motor4.currentDistance();
    motor4JSON["distance_error"] = motor4.currentDistanceError();
    motor4JSON["distance_steering"] = motor4.currentDistanceSteering();
    motor4JSON["velocity"] = motor4.currentVelocity();
    motor4JSON["velocity_error"] = motor4.currentDistanceError();
    motor4JSON["velocity_steering"] = motor4.currentVelocityError();
    motor4JSON["steering"] = motor4.currentSteering();
    serializeJson(doc, Serial);
    Serial.println("");
  }

  motor1.loop();
  motor2.loop();
  motor3.loop();
  motor4.loop();
  command.parse();
  loopCount = (loopCount + 1) % 100;
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
