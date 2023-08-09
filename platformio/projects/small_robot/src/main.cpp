#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <LED.h>
#include <ServoWheel.h>
#include <JSONCommand.h>
#include <ArduinoQueue.h>

#include "config.h"
#include "message_utils.h"
#include "move_utils.h"


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

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#define GPSSerial Serial4
Adafruit_GPS GPS(&GPSSerial);

Servo servoPan;
Servo servoTilt;
Led redLed = Led(USER_LED_1);
Led greenLed = Led(USER_LED_2, true);

MessageParser messageParser(&moveQueue, &motor1, &motor2, &motor3, &motor4);
SensorHandler sensorHandler(&moveQueue, &bno, &GPS, &motor1, &motor2, &motor3, &motor4);

void commandHandler(char* input);
JSONCommand command = JSONCommand("\n", commandHandler);

void commandHandler(char* input){
  messageParser.parse(input).printJson();
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

  sensorHandler.setup(BATT_MEAS_AIN, SERV_PAN, SERV_TILT);

  pinMode(USER_BUTTON_1, INPUT);
  pinMode(USER_BUTTON_2, INPUT);
}

void serialEvent4() {
  GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
}

void loop(void){
  command.parse();
  sensorHandler.printOutput();

  if(sensorHandler.motorsReady()){
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

  motor1.loop();
  motor2.loop();
  motor3.loop();
  motor4.loop();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
