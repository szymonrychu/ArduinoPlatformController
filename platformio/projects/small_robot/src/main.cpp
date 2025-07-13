#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <LED.h>
#include <VelocityController.h>
#include <JSONCommand.h>
#include <ArduinoQueue.h>
#include <Battery.h>

#include "config.h"

enum OutputMessageType {
    OUTPUT_UNDEFINED,
    OUTPUT_SUCCESS,
    OUTPUT_ERROR
};
class OutputMessage{
private:
  OutputMessageType type;
  char* message;
public:
  OutputMessage(OutputMessageType type, char* message){
    this->type = type;
    this->message = message;
  }

  void printJson(){
    StaticJsonDocument<128> out;
    out["micros"] = micros();
    switch(this->type){
    case OUTPUT_SUCCESS:
        out["message_type"] = "SUCCESS";
        break;
    case OUTPUT_ERROR:
        out["message_type"] = "ERROR";
        break;
    default:
        out["message_type"] = "UNDEFINED";
        break;
    }
    
    out["message"] = this->message;
    serializeJson(out, Serial);
    Serial.println("");
  }

  bool isSuccess(){
    return this->type == OUTPUT_SUCCESS;
  }
};


#define QUEUE_SIZE_ITEMS 10
 
#define BNO055_SAMPLERATE_DELAY_MS (10)

#define MEM_LEN 1024
char buffer[MEM_LEN];
long loopCounter = 0;
double batteryVoltage = 0;

VelocityController motor1(1, M1_ENCA, M1_ENCB, M1_HA, M1_HB, M1_SERV);
VelocityController motor2(2, M2_ENCA, M2_ENCB, M2_HA, M2_HB, M2_SERV);
VelocityController motor3(3, M3_ENCA, M3_ENCB, M3_HA, M3_HB, M3_SERV);
VelocityController motor4(4, M4_ENCA, M4_ENCB, M4_HA, M4_HB, M4_SERV);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#define GPSSerial Serial4
Adafruit_GPS gps(&GPSSerial);
bool gpsReceiving = false;
bool gpsParsing = false;
bool showPIDs = true;

ServoController servoPan(SERV_PAN);
ServoController servoTilt(SERV_TILT);

Led redLed = Led(USER_LED_1);
Led greenLed = Led(USER_LED_2);

Battery battery;

void commandHandler(char* input);
JSONCommand command = JSONCommand("\n", commandHandler);

uint64_t lastCommandTimestampMicros = 0;
uint64_t moveTimeout = 0;

char* moveUuid = "";

void commandHandler(char* input){
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, input);
  JsonObject obj = doc.as<JsonObject>();

  double moveDuration = 0;
  lastCommandTimestampMicros = micros();

  if(!obj["duration"].isNull()){
    moveDuration = obj["duration"];
    moveTimeout = (uint64_t)(1000000.0 * moveDuration) + lastCommandTimestampMicros;
  } else {
    moveTimeout = 0;
  }

  if(!obj["moveUuid"].isNull()){
    moveUuid = String(obj["moveUuid"]).c_str();
  }

  bool resetMotorsControllers = true;

  if(!obj["motor1"].isNull()){
    if(!obj["motor1"]["velocity"].isNull()){
      motor1.drive(obj["motor1"]["velocity"]);
      resetMotorsControllers = false;
    }
    if(!obj["motor1"]["kP"].isNull()){
      motor1.setKP(obj["motor1"]["kP"]);
      showPIDs = true;
    }
    if(!obj["motor1"]["kI"].isNull()){
      motor1.setKI(obj["motor1"]["kI"]);
      showPIDs = true;
    }
    if(!obj["motor1"]["kD"].isNull()){
      motor1.setKD(obj["motor1"]["kD"]);
      showPIDs = true;
    }
  }
  if(!obj["servo1"].isNull()){
    if(!obj["servo1"]["angle"].isNull()){
      motor1.writeServo(obj["servo1"]["angle"], moveDuration);
    }
  }
  if(!obj["motor2"].isNull()){
    if(!obj["motor2"]["velocity"].isNull()){
      motor2.drive(obj["motor2"]["velocity"]);
      resetMotorsControllers = false;
    }
    if(!obj["motor2"]["kP"].isNull()){
      motor2.setKP(obj["motor2"]["kP"]);
      showPIDs = true;
    }
    if(!obj["motor2"]["kI"].isNull()){
      motor2.setKI(obj["motor2"]["kI"]);
      showPIDs = true;
    }
    if(!obj["motor2"]["kD"].isNull()){
      motor2.setKD(obj["motor2"]["kD"]);
      showPIDs = true;
    }
  }
  if(!obj["servo2"].isNull()){
    if(!obj["servo2"]["angle"].isNull()){
      motor2.writeServo(obj["servo2"]["angle"], moveDuration);
    }
  }
  if(!obj["motor3"].isNull()){
    if(!obj["motor3"]["velocity"].isNull()){
      motor3.drive(obj["motor3"]["velocity"]);
      resetMotorsControllers = false;
    }
    if(!obj["motor3"]["kP"].isNull()){
      motor3.setKP(obj["motor3"]["kP"]);
      showPIDs = true;
    }
    if(!obj["motor3"]["kI"].isNull()){
      motor3.setKI(obj["motor3"]["kI"]);
      showPIDs = true;
    }
    if(!obj["motor3"]["kD"].isNull()){
      motor3.setKD(obj["motor3"]["kD"]);
      showPIDs = true;
    }
  }
  if(!obj["servo3"].isNull()){
    if(!obj["servo3"]["angle"].isNull()){
      motor3.writeServo(obj["servo3"]["angle"], moveDuration);
    }
  }
  if(!obj["motor4"].isNull()){
    if(!obj["motor4"]["velocity"].isNull()){
      motor4.drive(obj["motor4"]["velocity"]);
      resetMotorsControllers = false;
    }
    if(!obj["motor4"]["kP"].isNull()){
      motor4.setKP(obj["motor4"]["kP"]);
      showPIDs = true;
    }
    if(!obj["motor4"]["kI"].isNull()){
      motor4.setKI(obj["motor4"]["kI"]);
      showPIDs = true;
    }
    if(!obj["motor4"]["kD"].isNull()){
      motor4.setKD(obj["motor4"]["kD"]);
      showPIDs = true;
    }
  }
  if(!obj["servo4"].isNull()){
    if(!obj["servo4"]["angle"].isNull()){
      motor4.writeServo(obj["servo4"]["angle"], moveDuration);
    }
  }
  if(resetMotorsControllers){
    motor1.resetDistanceVelocity();
    motor2.resetDistanceVelocity();
    motor3.resetDistanceVelocity();
    motor4.resetDistanceVelocity();
  }

  if(!obj["pan"].isNull()){
    if(!obj["pan"]["angle"].isNull()) servoPan.writeServo(obj["pan"]["angle"], moveDuration);
  }

  if(!obj["tilt"].isNull()){
    if(!obj["tilt"]["angle"].isNull()) servoTilt.writeServo(obj["tilt"]["angle"], moveDuration);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    command.inputChar(ch);
  }
}

void setup(void){
  Serial.begin(115200);

  while(!Serial){delay(10);}

  motor1.setup();
  motor2.setup();
  motor3.setup();
  motor4.setup();
  servoPan.setup();
  servoTilt.setup();

  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  gps.sendCommand(PGCMD_ANTENNA);

  while(!bno.begin()) {
    OutputMessage(OUTPUT_ERROR, "BNO055 init error").printJson();
    delay(1000);
  }
  bno.setExtCrystalUse(true);
  Wire.setClock(400000); // 400KHz

  battery.setup(BATT_MEAS_AIN);

  pinMode(USER_BUTTON_1, INPUT);
  pinMode(USER_BUTTON_2, INPUT);
  greenLed.setState(true);
}

void serialEvent4() {
  gps.read();
  if (gps.newNMEAreceived()) {
    gpsReceiving = true;
    gpsParsing = gps.parse(gps.lastNMEA()) || gpsParsing;
  }
}

void loop(void){
  command.parse();

  uint64_t currentTimeMicros = micros();
  bool moveEnded = moveTimeout > 0 && moveTimeout < currentTimeMicros;

  if(moveEnded) {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    motor1.resetDistanceVelocity();
    motor2.resetDistanceVelocity();
    motor3.resetDistanceVelocity();
    motor4.resetDistanceVelocity();
  }

  motor1.loop(currentTimeMicros);
  motor2.loop(currentTimeMicros);
  motor3.loop(currentTimeMicros);
  motor4.loop(currentTimeMicros);

  servoPan.loop(currentTimeMicros);
  servoTilt.loop(currentTimeMicros);

  sensors_event_t angVelocityData , accData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  int8_t temp = bno.getTemp();

  loopCounter = (loopCounter+1)%10;
  if(loopCounter == 0){
    StaticJsonDocument<1024> doc;
    doc["micros"] = currentTimeMicros;
    doc["temp"] = temp;
    if(moveUuid != "") {
      doc["moveUuid"] = moveUuid;
    }

    JsonObject batt = doc.createNestedObject("battery");
    batt["voltage"] = battery.readVoltage();

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

    JsonObject gps_ = doc.createNestedObject("gps");
    gps_["receiving"] = gpsReceiving;
    gps_["parsing"] = gpsParsing;
    gps_["fix"] = gps.fix;
    gps_["fix_quality"] = gps.fixquality;
    gps_["satellites"] = gps.satellites;
    gps_["speed"] = gps.speed;
    gps_["angle"] = gps.angle;
    gps_["altitude"] = gps.altitude;

    float gpsDecimalLatitude = 0.0;
    float gpsDecimalLongitude = 0.0;
    int gps_DD_latitude = int(gps.latitude/100.0);
    float gps_MMMM_latitude = gps.latitude - 100.0*gps_DD_latitude;
    gpsDecimalLatitude = gps_DD_latitude + float(gps_MMMM_latitude)/60.0;
    int gps_DDD_longitude = int(gps.longitude/100.0);
    float gps_MMMM_longitude = gps.longitude - 100.0*gps_DDD_longitude;
    gpsDecimalLongitude = gps_DDD_longitude + float(gps_MMMM_longitude)/60.0;
    gps_["dec_latitude"] = gpsDecimalLatitude;
    gps_["dec_longitude"] = gpsDecimalLongitude;

    JsonObject m1 = doc.createNestedObject("motor1");
    m1["velocity"] = motor1.currentVelocity();
    m1["distance"] = motor1.currentDistanceDelta();
    if(showPIDs){
      JsonObject pid1 = m1.createNestedObject("PID");
      pid1["kP"] = motor1.getKP();
      pid1["kI"] = motor1.getKI();
      pid1["kD"] = motor1.getKD();
    }
    JsonObject s1 = doc.createNestedObject("servo1");
    s1["angle"] = motor1.readServo();

    JsonObject m2 = doc.createNestedObject("motor2");
    m2["velocity"] = motor2.currentVelocity();
    m2["distance"] = motor2.currentDistanceDelta();
    if(showPIDs){
      JsonObject pid2 = m2.createNestedObject("PID");
      pid2["kP"] = motor2.getKP();
      pid2["kI"] = motor2.getKI();
      pid2["kD"] = motor2.getKD();
    }
    JsonObject s2 = doc.createNestedObject("servo2");
    s2["angle"] = motor2.readServo();

    JsonObject m3 = doc.createNestedObject("motor3");
    m3["velocity"] = motor3.currentVelocity();
    m3["distance"] = motor3.currentDistanceDelta();
    if(showPIDs){
      JsonObject pid3 = m3.createNestedObject("PID");
      pid3["kP"] = motor3.getKP();
      pid3["kI"] = motor3.getKI();
      pid3["kD"] = motor3.getKD();
    }
    JsonObject s3 = doc.createNestedObject("servo3");
    s3["angle"] = motor3.readServo();

    JsonObject m4 = doc.createNestedObject("motor4");
    m4["velocity"] = motor4.currentVelocity();
    m4["distance"] = motor4.currentDistanceDelta();
    if(showPIDs){
      JsonObject pid4 = m4.createNestedObject("PID");
      pid4["kP"] = motor4.getKP();
      pid4["kI"] = motor4.getKI();
      pid4["kD"] = motor4.getKD();
    }
    JsonObject s4 = doc.createNestedObject("servo4");
    s4["angle"] = motor4.readServo();

    JsonObject pan = doc.createNestedObject("pan");
    pan["angle"] = servoPan.readServo();

    JsonObject tilt = doc.createNestedObject("tilt");
    tilt["angle"] = servoTilt.readServo();

    serializeJson(doc, Serial);
    Serial.println("");

    if(showPIDs) showPIDs = false;
    if(!moveEnded) redLed.toggle();
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
