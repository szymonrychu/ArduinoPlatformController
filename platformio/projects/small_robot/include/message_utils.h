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
        out["type"] = "SUCCESS";
        break;
    case OUTPUT_ERROR:
        out["type"] = "ERROR";
        break;
    default:
        out["type"] = "UNDEFINED";
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
        if(obj["type"] == "reset") return INPUT_RESET;
        if(obj["type"] == "reset_queue") return INPUT_RESET_QUEUE;
        if(obj["type"] == "stop") return INPUT_STOP;
        if(obj["type"] == "raw_move") return INPUT_RAW_MOVE;
        if(obj["type"] == "turn") return INPUT_TURN_MOVE;
        if(obj["type"] == "forward") return INPUT_FORWARD_MOVE;
        if(obj["type"] == "sequentional_move") return INPUT_SEQUENTIONAL_MOVE;
        return INPUT_UNDEFINED;
    }

    /*
{"type":"reset_queue"}
    */
    OutputMessage resetQueue(){
        while(!moveQueue->isEmpty()){
            moveQueue->dequeue();
        }
        return OutputMessage(OUTPUT_SUCCESS, "reset_queue ok");
    }

    /*
{"type":"stop"}
    */
    OutputMessage resetMotors(){
        motor1->resetDistanceVelocity();
        motor2->resetDistanceVelocity();
        motor3->resetDistanceVelocity();
        motor4->resetDistanceVelocity();
        return OutputMessage(OUTPUT_SUCCESS, "stop_motors ok");
    }

    /*
{"type":"reset"}
    */
    OutputMessage resetAll(){
        this->resetQueue();
        this->resetMotors();
        return OutputMessage(OUTPUT_SUCCESS, "reset_all ok");
    }

    /*

{"type":"raw_move","motor1":{"distance":0.2,"velocity":0.5},"motor2":{"distance":0.2,"velocity":0.5},"motor3":{"distance":0.2,"velocity":0.5},"motor4":{"distance":0.2,"velocity":0.5}}
{"type":"raw_move","motor1":{"distance":-0.2,"velocity":0.5},"motor2":{"distance":-0.2,"velocity":0.5},"motor3":{"distance":-0.2,"velocity":0.5},"motor4":{"distance":-0.2,"velocity":0.5}}
{"type":"raw_move","motor1":{"distance":0,"velocity":0},"motor2":{"distance":0,"velocity":0},"motor3":{"distance":0,"velocity":0},"motor4":{"distance":0,"velocity":0}}
{"type":"raw_move","motor1":{"distance":1.0,"velocity":0.5},"motor2":{"distance":1.0,"velocity":0.5},"motor3":{"distance":1.0,"velocity":0.5},"motor4":{"distance":1.0,"velocity":0.5}}

{"type":"raw_move","motor1":{"distance":1.0,"velocity":0.5}}
{"type":"raw_move","motor2":{"distance":1.0,"velocity":0.5}}
{"type":"raw_move","motor3":{"distance":1.0,"velocity":0.5}}
{"type":"raw_move","motor4":{"distance":1.0,"velocity":0.5}}

{"type":"raw_move","motor1":{"angle":0.7853},"motor2":{"angle":0.7853},"motor3":{"angle":0.7853},"motor4":{"angle":0.7853}}
{"type":"raw_move","motor1":{"angle":-0.7853},"motor2":{"angle":-0.7853},"motor3":{"angle":-0.7853},"motor4":{"angle":-0.7853}}
{"type":"raw_move","motor1":{"angle":1.5706},"motor2":{"angle":1.5706},"motor3":{"angle":1.5706},"motor4":{"angle":1.5706}}
{"type":"raw_move","motor1":{"angle":0},"motor2":{"angle":0},"motor3":{"angle":0},"motor4":{"angle":0}}

{"type":"raw_move","pan":{"angle":0.7853},"tilt":{"angle":0.7853}}
{"type":"raw_move","pan":{"angle":0},"tilt":{"angle":0}}

{"type":"raw_move","motor1":{"distance":0.2,"velocity":0.5,"angle":0.7853},"motor2":{"distance":0.2,"velocity":0.5,"angle":0.7853},"motor3":{"distance":0.2,"velocity":0.5,"angle":0.7853},"motor4":{"distance":0.2,"velocity":0.5,"angle":0.7853},{"pan":{"angle":0.7853},"tilt":{"angle":0.7853}}}

    */
    
    OutputMessage parseRawMove(JsonObject& obj){
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
        
        moveQueue->enqueue(move);

        return OutputMessage(OUTPUT_SUCCESS, "raw_move ok");
    }


    /*
{"type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5}
{"type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5}
{"type":"turn", "turn_angle":0.7853, "turn_velocity": 0.4}
{"type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.4}

{"type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.6, "turn_x": 0.2, "turn_y": 0.2}

{"type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": 0.4}
{"type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": 0.4}
{"type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": -0.4}
{"type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": -0.4}

{"type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": 0.4, "turn_y": 0.4}
{"type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": 0.4, "turn_y": 0.4}
{"type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5, "turn_x": -0.4, "turn_y": 0.4}
{"type":"turn", "turn_angle":-0.7853, "turn_velocity": 0.5, "turn_x": -0.4, "turn_y": 0.4}
    */
    OutputMessage parseTurnMove(JsonObject& obj){
        double turnCenterX = obj["turn_x"];
        double turnCenterY = obj["turn_y"];

        double turnAngle = obj["turn_angle"];
        double turnVelocity = obj["turn_velocity"];

        Move move = turn(turnAngle, turnVelocity, turnCenterX, turnCenterY);
        
        moveQueue->enqueue(filterAngles(move));
        moveQueue->enqueue(filterDistanceVelocity(move));

        return OutputMessage(OUTPUT_SUCCESS, "turn_move ok");
    }

    /*
{"type":"forward", "move_distance":0.25, "move_velocity": 0.5}
{"type":"forward", "move_distance":-0.25, "move_velocity": 0.5}
{"type":"forward", "move_distance":0.25, "move_velocity": 0.5, "move_angle":0.7853}
{"type":"forward", "move_distance":0.25, "move_velocity": 0.5, "move_angle":-0.7853}
    */
    OutputMessage parseForwardMove(JsonObject& obj){
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

        moveQueue->enqueue(zeroAngles(moveAngle));
        moveQueue->enqueue(move);

        return OutputMessage(OUTPUT_SUCCESS, "move_forward ok");
    }

    /*
{"type":"sequentional_move","moves":[{"type":"turn", "turn_angle":0.7853, "turn_velocity": 0.5},{"type":"forward", "move_distance":0.25, "move_velocity": 0.5}]}
    */

    OutputMessage parseSequentionalMove(JsonObject& obj){
        JsonArray raw_moves = obj["moves"].as<JsonArray>();
        for(JsonObject move_obj : raw_moves){
            OutputMessage tmp(OUTPUT_SUCCESS, "placeholder");
            switch(this->parseInputMessageType(move_obj)){
            case INPUT_RAW_MOVE:
                tmp = this->parseRawMove(move_obj);
                break;
            case INPUT_TURN_MOVE:
                tmp = this->parseTurnMove(move_obj);
                break;
            case INPUT_FORWARD_MOVE:
                tmp = this->parseForwardMove(move_obj);
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
        case INPUT_RAW_MOVE:
            return this->parseRawMove(obj);
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

class Battery {
private:
    uint8_t batteryPin = 0;
public:

    void setup(uint8_t batteryPin){
        this->batteryPin = batteryPin;
        pinMode(this->batteryPin, INPUT);
    }

    double readVoltage(){
        return 10.82*((float)analogRead(this->batteryPin))/498.0;
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
            doc["type"] = "STATUS";
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
            if(!mReady){
                 JsonObject motor1JSON = doc.createNestedObject("motor1");
                motor1JSON["ready"] = motor1->ready();
                motor1JSON["servo"] = motor1->readServo();
                motor1JSON["distance"] = motor1->currentDistance();
                motor1JSON["distance_error"] = motor1->currentDistanceError();
                motor1JSON["distance_steering"] = motor1->currentDistanceSteering();
                motor1JSON["velocity"] = motor1->currentVelocity();
                motor1JSON["velocity_error"] = motor1->currentDistanceError();
                motor1JSON["velocity_steering"] = motor1->currentVelocityError();
                motor1JSON["steering"] = motor1->currentSteering();
                JsonObject motor2JSON = doc.createNestedObject("motor2");
                motor2JSON["ready"] = motor2->ready();
                motor2JSON["servo"] = motor2->readServo();
                motor2JSON["distance"] = motor2->currentDistance();
                motor2JSON["distance_error"] = motor2->currentDistanceError();
                motor2JSON["distance_steering"] = motor2->currentDistanceSteering();
                motor2JSON["velocity"] = motor2->currentVelocity();
                motor2JSON["velocity_error"] = motor2->currentDistanceError();
                motor2JSON["velocity_steering"] = motor2->currentVelocityError();
                motor2JSON["steering"] = motor2->currentSteering();
                JsonObject motor3JSON = doc.createNestedObject("motor3");
                motor3JSON["ready"] = motor3->ready();
                motor3JSON["servo"] = motor3->readServo();
                motor3JSON["distance"] = motor3->currentDistance();
                motor3JSON["distance_error"] = motor3->currentDistanceError();
                motor3JSON["distance_steering"] = motor3->currentDistanceSteering();
                motor3JSON["velocity"] = motor3->currentVelocity();
                motor3JSON["velocity_error"] = motor3->currentDistanceError();
                motor3JSON["velocity_steering"] = motor3->currentVelocityError();
                motor3JSON["steering"] = motor3->currentSteering();
                JsonObject motor4JSON = doc.createNestedObject("motor4");
                motor4JSON["ready"] = motor4->ready();
                motor4JSON["servo"] = motor4->readServo();
                motor4JSON["distance"] = motor4->currentDistance();
                motor4JSON["distance_error"] = motor4->currentDistanceError();
                motor4JSON["distance_steering"] = motor4->currentDistanceSteering();
                motor4JSON["velocity"] = motor4->currentVelocity();
                motor4JSON["velocity_error"] = motor4->currentDistanceError();
                motor4JSON["velocity_steering"] = motor4->currentVelocityError();
                motor4JSON["steering"] = motor4->currentSteering();
            }
            serializeJson(doc, Serial);
            Serial.println("");
        }
    }

};

#endif