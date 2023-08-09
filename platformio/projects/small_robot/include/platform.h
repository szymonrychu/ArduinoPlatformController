// #ifndef PLATFORM_H
// #define PLATFORM_H

// #include <Arduino.h>
// #include <Wire.h>
// #include <ArduinoJson.h>
// #include <ArduinoQueue.h>
// #include <ServoWheel.h>
// #include <vector.h>
// #include <Adafruit_BNO055.h>
// #include <Adafruit_GPS.h>
// #include <Servo.h>
// #include <Battery.h>

// #include "move_utils.h"

// class RobotPlatform {

// private:
//     ArduinoQueue<Move>* moveQueue = NULL;
//     ServoWheel motor1;
//     ServoWheel motor2;
//     ServoWheel motor3;
//     ServoWheel motor4;
//     Adafruit_GPS* gps;
//     Adafruit_BNO055* bno;
//     Battery battery;
//     Servo servoPan;
//     Servo servoTilt;
//     uint32_t loopCount = 0;
//     XYCoordinates currentPlatformCoordinates;

// public:
//     RobotPlatform(MotorConfig motor1, MotorConfig motor2, MotorConfig motor3, MotorConfig motor4, MoveQueue){
//         this->motor1 = ServoWheel(motor1);
//         this->motor2 = ServoWheel(motor2);
//         this->motor3 = ServoWheel(motor3);
//         this->motor4 = ServoWheel(motor4);
//         this->moveQueue = 
//     }



// };

// #endif