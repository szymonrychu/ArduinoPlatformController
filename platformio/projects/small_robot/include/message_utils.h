#ifndef MESSAGE_UTILS_H
#define MESSAGE_UTILS_H

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <ArduinoQueue.h>
#include <ServoWheel.h>
#include <vector.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Servo.h>
#include <Battery.h>

#include "move_utils.h"

enum OutputMessageType {
    OUTPUT_UNDEFINED,
    OUTPUT_SUCCESS,
    OUTPUT_ERROR
};

enum InputMessageType {
    INPUT_UNDEFINED,
    INPUT_RESET,
    INPUT_RESET_QUEUE,
    INPUT_STOP,
    INPUT_RAW_MOVE,
    INPUT_TURN_MOVE,
    INPUT_FORWARD_MOVE,
    INPUT_SEQUENTIONAL_MOVE
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

class MessageParser{
private:
    ArduinoQueue<Move>* moveQueue = NULL;
    ServoWheel* motor1;
    ServoWheel* motor2;
    ServoWheel* motor3;
    ServoWheel* motor4;

    InputMessageType parseInputMessageType(JsonObject& obj){
        if(obj["message_type"] == "reset") return INPUT_RESET;
        if(obj["message_type"] == "reset_queue") return INPUT_RESET_QUEUE;
        if(obj["message_type"] == "stop") return INPUT_STOP;
        if(obj["message_type"] == "raw_move") return INPUT_RAW_MOVE;
        if(obj["message_type"] == "turn") return INPUT_TURN_MOVE;
        if(obj["message_type"] == "forward") return INPUT_FORWARD_MOVE;
        if(obj["message_type"] == "sequentional_move") return INPUT_SEQUENTIONAL_MOVE;
        return INPUT_UNDEFINED;
    }

    /*
{"message_type":"reset_queue"}
    */
    OutputMessage resetQueue(){
        while(!moveQueue->isEmpty()){
            moveQueue->dequeue();
        }
        return OutputMessage(OUTPUT_SUCCESS, "reset_queue ok");
    }

    /*
{"message_type":"stop"}
    */
    OutputMessage resetMotors(){
        motor1->resetDistanceVelocity();
        motor2->resetDistanceVelocity();
        motor3->resetDistanceVelocity();
        motor4->resetDistanceVelocity();
        return OutputMessage(OUTPUT_SUCCESS, "stop_motors ok");
    }

    /*
{"message_type":"reset"}
    */
    OutputMessage resetAll(){
        this->resetQueue();
        this->resetMotors();
        return OutputMessage(OUTPUT_SUCCESS, "reset_all ok");
    }

    /*
{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "UUID":"xd"}
{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "UUID":"xd"}
{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.4, "UUID":"xd"}
{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.4, "UUID":"xd"}

{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.6, "turn_x": 0.2, "turn_y": 0.2, "UUID":"xd"}

{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": 0.4, "UUID":"xd"}
{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": 0.4, "UUID":"xd"}
{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": -0.4, "UUID":"xd"}
{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": -0.4, "UUID":"xd"}

{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": 0.4, "turn_y": 0.4, "UUID":"xd"}
{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": 0.4, "turn_y": 0.4, "UUID":"xd"}
{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": -0.4, "turn_y": 0.4, "UUID":"xd"}
{"message_type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": -0.4, "turn_y": 0.4, "UUID":"xd"}
    */
    OutputMessage parseTurnMove(JsonObject& obj){
        String moveUUID = obj["UUID"];
        if(moveUUID.length() < 1) return OutputMessage(OUTPUT_ERROR, "turn_move missing UUID");

        double turnCenterX = obj["turn_x"];
        double turnCenterY = obj["turn_y"];

        double turnAngle = obj["turn_angle"];
        double turnVelocity = obj["turn_velocity"];

        Move move = turn(turnAngle, turnVelocity, turnCenterX, turnCenterY);

        move.moveUUID = moveUUID;
        
        moveQueue->enqueue(filterAngles(move, 0, 2));
        moveQueue->enqueue(filterDistanceVelocity(move, 1, 2));

        return OutputMessage(OUTPUT_SUCCESS, "turn_move ok");
    }

    /*
{"message_type":"forward", "move_distance":0.25, "move_velocity": 0.5, "UUID":"xd"}
{"message_type":"forward", "move_distance":-0.25, "move_velocity": 0.5, "UUID":"xd"}
{"message_type":"forward", "move_distance":0.25, "move_velocity": 0.5, "move_angle":0.7853, "UUID":"xd"}
{"message_type":"forward", "move_distance":0.25, "move_velocity": 0.5, "move_angle":-0.7853, "UUID":"xd"}
    */
    OutputMessage parseForwardMove(JsonObject& obj){
        String moveUUID = obj["UUID"];
        if(moveUUID.length() < 1) return OutputMessage(OUTPUT_ERROR, "move_forward missing UUID");

        double moveDistance = obj["move_distance"];
        double moveVelocity = obj["move_velocity"];
        double moveAngle = obj["move_angle"];
        
        Move move;
        move.motor1.distanceSet = true;
        move.motor1.distanceDelta = moveDistance;
        move.motor1.velocity = moveVelocity;
        
        move.motor2.distanceSet = true;
        move.motor2.distanceDelta = moveDistance;
        move.motor2.velocity = moveVelocity;
        
        move.motor3.distanceSet = true;
        move.motor3.distanceDelta = moveDistance;
        move.motor3.velocity = moveVelocity;
        
        move.motor4.distanceSet = true;
        move.motor4.distanceDelta = moveDistance;
        move.motor4.velocity = moveVelocity;

        move.moveUUID = moveUUID;

        moveQueue->enqueue(zeroAngles(moveAngle, 0, 2));

        move.movePart = 1;
        move.maxMoveParts = 2;
        moveQueue->enqueue(move);

        return OutputMessage(OUTPUT_SUCCESS, "move_forward ok");
    }

    /*
{"message_type":"sequentional_move","moves":[{"message_type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "UUID":"1"},{"message_type":"forward", "move_distance":0.25, "move_velocity": 0.5, "UUID":"2"}]}
    */

    OutputMessage parseSequentionalMove(JsonObject& obj){
        JsonArray raw_moves = obj["moves"].as<JsonArray>();
        for(JsonObject move_obj : raw_moves){
            OutputMessage tmp(OUTPUT_SUCCESS, "placeholder");
            switch(this->parseInputMessageType(move_obj)){
                break;
            case INPUT_TURN_MOVE:
                tmp = this->parseTurnMove(move_obj);
                break;
            case INPUT_FORWARD_MOVE:
                tmp = this->parseForwardMove(move_obj);
                break;
            case INPUT_STOP:
                tmp = this->resetMotors();
                break;
            case INPUT_RESET:
                tmp = this->resetAll();
                break;
            case INPUT_RESET_QUEUE:
                tmp = this->resetQueue();
                break;
            default:
                break;
            }
            if(!tmp.isSuccess()){
                return tmp;
            }
        }
        return OutputMessage(OUTPUT_SUCCESS, "sequentional_move ok");
    }

public:
    MessageParser(ArduinoQueue<Move>* mQueue, ServoWheel* motor1, ServoWheel* motor2, ServoWheel* motor3, ServoWheel* motor4){
        this->moveQueue = mQueue;
        this->motor1 = motor1;
        this->motor2 = motor2;
        this->motor3 = motor3;
        this->motor4 = motor4;
    }

    OutputMessage parse(char* input){
        DynamicJsonDocument doc(512);
        deserializeJson(doc, input);
        JsonObject obj = doc.as<JsonObject>();

        switch(this->parseInputMessageType(obj)){
        case INPUT_RESET:
            return this->resetAll();
        case INPUT_STOP:
            return this->resetMotors();
        case INPUT_RESET_QUEUE:
            return this->resetQueue();
        case INPUT_TURN_MOVE:
            return this->parseTurnMove(obj);
        case INPUT_FORWARD_MOVE:
            return this->parseForwardMove(obj);
        case INPUT_SEQUENTIONAL_MOVE:
            return this->parseSequentionalMove(obj);
        default:
            return OutputMessage(OUTPUT_ERROR, "wrong message type");
        }
    }
};

class SensorHandler {
private:
    ArduinoQueue<Move>* moveQueue = NULL;
    ServoWheel* motor1;
    ServoWheel* motor2;
    ServoWheel* motor3;
    ServoWheel* motor4;
    Adafruit_GPS* gps;
    Adafruit_BNO055* bno;
    Battery battery;
    Servo servoPan;
    Servo servoTilt;
    uint32_t loopCount = 0;
    Move currentMove;
    bool currentMoveDone_ = true;

public:
    SensorHandler(ArduinoQueue<Move>* mQueue, Adafruit_BNO055* bno, Adafruit_GPS* gps,
        ServoWheel* motor1, ServoWheel* motor2, ServoWheel* motor3, ServoWheel* motor4){
        this->moveQueue = mQueue;
        this->motor1 = motor1;
        this->motor2 = motor2;
        this->motor3 = motor3;
        this->motor4 = motor4;
        this->gps = gps;
        this->bno = bno;
    }

    bool motorsReady(){
        return this->motor1->ready() && this->motor2->ready() && this->motor3->ready() && this->motor4->ready();
    }

    void setup(uint8_t batteryPin, uint8_t servoPanPin, uint8_t servoTiltPin){
        this->gps->begin(9600);
        this->gps->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        this->gps->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

        while(!bno->begin()) {
            OutputMessage(OUTPUT_ERROR, "BNO055 init error").printJson();
            delay(1000);
        }
        bno->setExtCrystalUse(true);
        Wire.setClock(400000); // 400KHz
        this->battery.setup(batteryPin);

        servoPan.attach(servoPanPin);
        servoTilt.attach(servoTiltPin);
    }

    void updateCurrentMove(Move m){
        this->currentMove = m;
        this->currentMoveDone_ = false;
    }

    void currentMoveDone(){
        currentMoveDone_ = true;
    }

    void printOutput(){
        bool mReady = motorsReady();

        sensors_event_t angVelocityData , accData;
        bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno->getEvent(&accData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Quaternion quat = bno->getQuat();
        int8_t temp = bno->getTemp();

        loopCount = (loopCount + 1) % 100;
        if(loopCount == 0 || !mReady){
            StaticJsonDocument<1536> doc;
            doc["micros"] = micros();
            doc["message_type"] = "STATUS";
            if(mReady){
                doc["status"] = "ready";
            }else{
                doc["status"] = "busy";
            }
            doc["queue_l"] = moveQueue->item_count();
            doc["int_temp"] = temp;
            JsonObject battery = doc.createNestedObject("battery");
            battery["voltage"] = this->battery.readVoltage();
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

            if(this->gps->fix){
                JsonObject gps = doc.createNestedObject("gps");
                gps["fix_quality"] = this->gps->fixquality;
                gps["satellites"] = this->gps->satellites;
                gps["speed"] = this->gps->speed;
                gps["angle"] = this->gps->angle;
                gps["altitude"] = this->gps->altitude;

                float gpsDecimalLatitude = 0.0;
                float gpsDecimalLongitude = 0.0;

                int gps_DD_latitude = int(this->gps->latitude/100.0);
                float gps_MMMM_latitude = this->gps->latitude - 100.0*gps_DD_latitude;
                gpsDecimalLatitude = gps_DD_latitude + float(gps_MMMM_latitude)/60.0;

                int gps_DDD_longitude = int(this->gps->longitude/100.0);
                float gps_MMMM_longitude = this->gps->longitude - 100.0*gps_DDD_longitude;
                gpsDecimalLongitude = gps_DDD_longitude + float(gps_MMMM_longitude)/60.0;

                gps["dec_latitude"] = gpsDecimalLatitude;
                gps["dec_longitude"] = gpsDecimalLongitude;
            }

            if(!currentMoveDone_){
                JsonObject moveProgress = doc.createNestedObject("move_progress");
                double motor1Progress = motor1->moveProgress();
                double motor2Progress = motor2->moveProgress();
                double motor3Progress = motor3->moveProgress();
                double motor4Progress = motor4->moveProgress();
                moveProgress["progress"] = (motor1Progress + motor2Progress + motor3Progress + motor4Progress)/4.0;
                moveProgress["uuid"] = currentMove.moveUUID;
                moveProgress["part"] = currentMove.movePart;
                moveProgress["max_parts"] = currentMove.maxMoveParts;
            }



            // if(!mReady){
            //     JsonObject motor1JSON = doc.createNestedObject("motor1");
            //     motor1JSON["ready"] = motor1->ready();
            //     motor1JSON["servo"] = motor1->readServo();
            //     motor1JSON["distance"] = motor1->currentDistance();
            //     motor1JSON["distance_error"] = motor1->currentDistanceError();
            //     motor1JSON["distance_steering"] = motor1->currentDistanceSteering();
            //     motor1JSON["velocity"] = motor1->currentVelocity();
            //     motor1JSON["velocity_error"] = motor1->currentDistanceError();
            //     motor1JSON["velocity_steering"] = motor1->currentVelocityError();
            //     motor1JSON["steering"] = motor1->currentSteering();
            //     JsonObject motor2JSON = doc.createNestedObject("motor2");
            //     motor2JSON["ready"] = motor2->ready();
            //     motor2JSON["servo"] = motor2->readServo();
            //     motor2JSON["distance"] = motor2->currentDistance();
            //     motor2JSON["distance_error"] = motor2->currentDistanceError();
            //     motor2JSON["distance_steering"] = motor2->currentDistanceSteering();
            //     motor2JSON["velocity"] = motor2->currentVelocity();
            //     motor2JSON["velocity_error"] = motor2->currentDistanceError();
            //     motor2JSON["velocity_steering"] = motor2->currentVelocityError();
            //     motor2JSON["steering"] = motor2->currentSteering();
            //     JsonObject motor3JSON = doc.createNestedObject("motor3");
            //     motor3JSON["ready"] = motor3->ready();
            //     motor3JSON["servo"] = motor3->readServo();
            //     motor3JSON["distance"] = motor3->currentDistance();
            //     motor3JSON["distance_error"] = motor3->currentDistanceError();
            //     motor3JSON["distance_steering"] = motor3->currentDistanceSteering();
            //     motor3JSON["velocity"] = motor3->currentVelocity();
            //     motor3JSON["velocity_error"] = motor3->currentDistanceError();
            //     motor3JSON["velocity_steering"] = motor3->currentVelocityError();
            //     motor3JSON["steering"] = motor3->currentSteering();
            //     JsonObject motor4JSON = doc.createNestedObject("motor4");
            //     motor4JSON["ready"] = motor4->ready();
            //     motor4JSON["servo"] = motor4->readServo();
            //     motor4JSON["distance"] = motor4->currentDistance();
            //     motor4JSON["distance_error"] = motor4->currentDistanceError();
            //     motor4JSON["distance_steering"] = motor4->currentDistanceSteering();
            //     motor4JSON["velocity"] = motor4->currentVelocity();
            //     motor4JSON["velocity_error"] = motor4->currentDistanceError();
            //     motor4JSON["velocity_steering"] = motor4->currentVelocityError();
            //     motor4JSON["steering"] = motor4->currentSteering();
            // }
            serializeJson(doc, Serial);
            Serial.println("");
        }
    }

};

#endif