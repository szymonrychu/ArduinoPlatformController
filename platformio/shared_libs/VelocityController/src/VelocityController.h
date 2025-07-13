#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#ifndef MAX_ENCODER_VALUE
#define MAX_ENCODER_VALUE 850*12
#endif

#ifndef REVOLUTION_TICK_COUNT
#define REVOLUTION_TICK_COUNT MAX_ENCODER_VALUE
#endif

#ifndef REVOLUTION_TO_DIST
#define REVOLUTION_TO_DIST 0.48f
#endif

#ifndef SERVO_FULL_ROTATION_UPDATE_SPEED
#define SERVO_FULL_ROTATION_UPDATE_SPEED PI/(0.18*3) // 0.18s/60deg
#endif

#ifndef VELOCITY_CONTROLLER_P
#define VELOCITY_CONTROLLER_P 1.0
#endif

#ifndef VELOCITY_CONTROLLER_I
#define VELOCITY_CONTROLLER_I 0.0 // 0.001 * 0.001 * 0.001 *0.001
#endif

#ifndef VELOCITY_CONTROLLER_D
#define VELOCITY_CONTROLLER_D 0.2
#endif

#define SERVO_STARTING_OFFSET PI/2
#define PID_MAX_INTEGRAL 1.0f

#include <Arduino.h>
#include <Servo.h>
#include <Cytron_MDD3A.h>
#include <HWQuadEncoder.h>
#include <PID.h>
#include <math.h>
#include <Ramp.h>

class ServoController {
private:
  Servo servo;
  RampDouble ramp;
  uint8_t servoPin = 255;
  double servoValueRadians = 0.0;

public:
  ServoController(){};

  ServoController(uint8_t servoPin){
    this->servoPin = servoPin;
  }

  void setup(){
    this->servo.attach(this->servoPin);
  }

  void loop(uint64_t currentTimeMicros=0){
    this->ramp.update();
    this->servoValueRadians = this->ramp.getValue()*PI;

    if(this->servoValueRadians > PI/2) this->servoValueRadians = PI/2;
    if(this->servoValueRadians < -PI/2) this->servoValueRadians = -PI/2;
    
    this->servo.writeMicroseconds(1500.0 + 2000.0*(this->servoValueRadians/PI));
  }

  bool servoReady(){
    return this->ramp.isFinished();
  }

  bool writeServo(double angle, double timeS=0){
    if(angle < -PI/2) angle = -PI/2;
    if(angle > PI/2) angle = PI/2;
    double angleDelta = abs(angle - this->servoValueRadians);
    if(angleDelta < 0.001) return true;
    double minTimeS = angleDelta*SERVO_FULL_ROTATION_UPDATE_SPEED/PI;
    if(timeS == 0){
      timeS = minTimeS;
    } else {
      if(timeS < minTimeS){
        double angleCoefficient = timeS/minTimeS;
        angle = angle * angleCoefficient;
      }
    }
    unsigned long timeMs = (unsigned long)(timeS * 1000.0);
    this->ramp.go(angle/PI, timeMs, LINEAR);
    return true;
  }

  double readServo(){
    return this->servoValueRadians;
  }

  void stop(){
    this->ramp.go(ramp.getValue());
  }
};

class VelocityController {
private:
  QuadEncoder encoder;
  MDD3AHbridge hbridge;
  PID velocityPID;
  ServoController servo;

  double distance = 0;
  double previousDistance = 0;
  double lastDistance = 0;
  double velocity = 0;
  double velocityTarget = 0;

  double steering = 0;

  uint64_t lastComputeTimeMicros = 0;
  uint64_t lastCommandTimestampMicros = 0;

  bool isFresh_ = true;
  bool initialised = false;

  double computeDistance(){
    int32_t currentEncoderTicks = this->encoder.read();
    int32_t currentRevolution = this->encoder.getRevolution();
    return (REVOLUTION_TO_DIST/MAX_ENCODER_VALUE)*((double)currentRevolution + (double)currentEncoderTicks/(double)REVOLUTION_TICK_COUNT);
  }

public:
  VelocityController(){};
  
  VelocityController(uint8_t encChannel, uint8_t encoderAPin, uint8_t encoderBPin, uint8_t hBridgeAPin, uint8_t hBridgeBPin, uint8_t servoPin){
    this->encoder = QuadEncoder(encChannel, encoderAPin, encoderBPin);
    this->hbridge = MDD3AHbridge(hBridgeAPin, hBridgeBPin);
    this->servo = ServoController(servoPin);
  }

  void setup(double P=VELOCITY_CONTROLLER_P, double I=VELOCITY_CONTROLLER_I, double D=VELOCITY_CONTROLLER_D){
    this->encoder.setInitConfig();
    this->encoder.init();
    this->hbridge.setup();
    this->velocityPID = PID(P, I, D);
    this->lastComputeTimeMicros = micros();
    this->servo.setup();
  }

  void loop(uint64_t currentTimeMicros=0){
    this->servo.loop(currentTimeMicros);
    if(currentTimeMicros==0){
      currentTimeMicros = micros();
    }

    this->distance = computeDistance();
    double timeDeltaSeconds =((double)currentTimeMicros - (double)this->lastComputeTimeMicros)/1000000.0;
    double lastDistanceDelta = this->distance - this->lastDistance;
    this->velocity = lastDistanceDelta/timeDeltaSeconds;

    this->velocityPID.setResult(this->velocity);

    this->steering = this->velocityPID.computeNewSteering(timeDeltaSeconds, this->velocityTarget);
    this->hbridge.drive(this->steering);


    this->lastDistance = this->distance;
    this->lastComputeTimeMicros = currentTimeMicros;
  }

  void drive(double velocity){
    this->velocityTarget = velocity;
    this->isFresh_ = false;
  }

  void stop(){
    this->velocityTarget = 0;
    this->servo.stop();
  }

  bool isStopped(){
    return 0.0f == this->velocityTarget;
  }

  double currentDistance(){
    return this->distance;
  }

  double currentDistanceDelta(){
    double currentDistance = this->currentDistance();
    double currentDistanceDelta = currentDistance - this->previousDistance;
    this->previousDistance = currentDistance;
    return currentDistanceDelta;
  }

  double currentVelocity(){
    return this->velocity;
  }

  double currentVelocityError(){
    return this->velocityPID.getError();
  }

  double currentSteering(){
    return this->steering;
  }

  bool isFresh(){
    return this->isFresh_;
  }

  bool servoReady(){
    return this->servo.servoReady();
  }

  void writeServo(double angle, double timeS=0){
    this->servo.writeServo(angle, timeS);
  }

  double readServo(){
    return this->servo.readServo();
  }

  void resetDistanceVelocity(){
    if(!this->isFresh_){
      this->lastDistance = this->distance;
      this->velocityPID.reset();
      this->velocity = 0;
      this->velocityTarget = 0;
      this->isFresh_ = true;
    }
  }

  float getKP(){
      return this->velocityPID.getKP();
  }

  void setKP(float kP){
      this->velocityPID.setKP(kP);
  }

  float getKI(){
      return this->velocityPID.getKI();
  }

  void setKI(float kI){
      this->velocityPID.setKI(kI);
  }

  float getKD(){
      return this->velocityPID.getKD();
  }
  
  void setKD(float kD){
      this->velocityPID.setKD(kD);
  }
};

#endif