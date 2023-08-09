#ifndef SERVO_WHEEL_H
#define SERVO_WHEEL_H

#ifndef MAX_ENCODER_VALUE
#define MAX_ENCODER_VALUE 850*12
#endif

#ifndef REVOLUTION_TICK_COUNT
#define REVOLUTION_TICK_COUNT MAX_ENCODER_VALUE
#endif

#ifndef REVOLUTION_TO_DIST
#define REVOLUTION_TO_DIST 0.48f
#endif

#ifndef MAX_RELATIVE_POWER
#define MAX_RELATIVE_POWER 1.0f
#endif

#ifndef DISTANCE_ERROR_ACCUMULATED_MAX_ABS
#define DISTANCE_ERROR_ACCUMULATED_MAX_ABS 1.0f
#endif

#ifndef VELOCITY_ERROR_ACCUMULATED_MAX_ABS
#define VELOCITY_ERROR_ACCUMULATED_MAX_ABS 1.0f
#endif

#ifndef MIN_COMMAND_MICROS
#define MIN_COMMAND_MICROS 250
#endif

#define SERVO_FULL_ROTATION_UPDATE_SPEED 1.2

#include <Arduino.h>
#include <Servo.h>
#include <Cytron_MDD3A.h>
#include <HWQuadEncoder.h>
#include <PID.h>
#include <math.h>

struct MotorMove {
  bool distanceSet = false;
  bool angleSet = false;
  double distanceDelta = 0;
  double velocity = 0;
  double angle = 0;
};
struct ServoMove {
  bool angleSet = false;
  double angle = 0;
};
struct Move {
  MotorMove motor1;
  MotorMove motor2;
  MotorMove motor3;
  MotorMove motor4;
  ServoMove pan;
  ServoMove tilt;
};

struct XYCoordinates{
  double X=0;
  double Y=0;
};

struct MotorConfig {
  uint8_t encoderChannel = 0;
  uint8_t encoderAPin = 0;
  uint8_t encoderBPin = 0;
  uint8_t hBridgeAPin = 0;
  uint8_t hBridgeBPin = 0;
  uint8_t servoPin = 0;
};


class ServoWheel{
private:
  QuadEncoder encoder;
  Servo servo;
  int servoPin = 255;
  MDD3AHbridge hbridge;
  PID distancePID;
  PID velocityPID;

  double distance = 0;
  double lastDistance = 0;
  double lastDistanceDelta = 0;
  double velocity = 0;
  uint64_t lastComputeTimeMicros = 0;

  uint64_t servoReadyAfterMicros = 0;
  double servoTarget = PI/4;
  double lastServoTarget = PI/4;

  double distanceTarget = 0;
  double lastDistanceTargetDelta = 0;
  double distanceSteering = 0;
  double lastMinimumDistanceError = 0;

  double velocityTarget = 0;
  double velocitySteering = 0;
  double steering = 0;

  bool distanceTargetSet = false;
  bool servoReady_ = true;
  bool isFresh_ = true;
  uint64_t lastCommandTimestampMicros = 0;

  XYCoordinates currentCoordinates;

  double computeDistance(){
    int32_t currentEncoderTicks = this->encoder.read();
    int32_t currentRevolution = this->encoder.getRevolution();
    return (REVOLUTION_TO_DIST/MAX_ENCODER_VALUE)*((double)currentRevolution + (double)currentEncoderTicks/(double)REVOLUTION_TICK_COUNT);
  }

public:
  ServoWheel(){};
  
  ServoWheel(int encChannel, int encoderAPin, int encoderBPin, int hBridgeAPin, int hBridgeBPin, int servoPin){
    this->encoder = QuadEncoder(encChannel, encoderAPin, encoderBPin);
    this->hbridge = MDD3AHbridge(hBridgeAPin, hBridgeBPin);
    this->servoPin = servoPin;
  }
  
  ServoWheel(MotorConfig config){
    this->encoder = QuadEncoder(config.encoderChannel, config.encoderAPin, config.encoderBPin);
    this->hbridge = MDD3AHbridge(config.hBridgeAPin, config.hBridgeBPin);
    this->servoPin = config.servoPin;
  }

  void setup(double dP=15.0, double dI=0, double dD=0.8, double vP=3.0, double vI=0.2, double vD=0.01){
    this->encoder.setInitConfig();
    this->encoder.init();
    this->hbridge.setup();
    this->servo.attach(this->servoPin);
    this->distancePID = PID(dP, dI, dD, -1.0, 1.0);
    this->velocityPID = PID(vP, vI, vD, 0.0, 1.0);
    this->lastComputeTimeMicros = micros();
    this->distanceTargetSet = false;
  }

  void loop(uint64_t currentTimeMicros=0){
    if(currentTimeMicros==0){
      currentTimeMicros = micros();
    }

    this->distance = computeDistance();
    double timeDeltaSeconds =((double)currentTimeMicros - (double)this->lastComputeTimeMicros)/1000000.0;
    this->lastDistanceDelta = this->distance - this->lastDistance;
    this->velocity = abs(lastDistanceDelta/timeDeltaSeconds);
  
    this->distancePID.setResult(this->distance);
    this->velocityPID.setResult(this->velocity);

    this->distanceSteering = this->distancePID.computeNewSteering(timeDeltaSeconds, this->distanceTarget);
    this->velocitySteering = this->velocityPID.computeNewSteering(timeDeltaSeconds, this->velocityTarget);

    this->steering = this->currentDistanceSteering() * this->currentVelocitySteering();

    this->hbridge.drive(this->steering);

    if(this->distanceTargetSet){
      if(abs(this->steering) < abs(this->currentVelocitySteering())){
        double absDistanceError = abs(this->distancePID.getError());
        if(abs(this->lastMinimumDistanceError - absDistanceError) < 0.01){
          this->distanceTargetSet = false;
          this->distancePID.reset();
          this->velocityTarget = 0.0;
        }else if(absDistanceError < this->lastMinimumDistanceError){
          this->lastMinimumDistanceError = absDistanceError;
        }
      }
    }

    this->lastDistance = this->distance;
    this->lastComputeTimeMicros = currentTimeMicros;
    if(this->servoTarget != this->lastServoTarget){
      this->servoReadyAfterMicros = currentTimeMicros + SERVO_FULL_ROTATION_UPDATE_SPEED*1000000.0;
      this->servo.writeMicroseconds(500.0 + 2000.0*(this->servoTarget/PI));
      this->lastServoTarget = this->servoTarget;
    }
    this->servoReady_ = this->servoReadyAfterMicros < currentTimeMicros;

    double deltaX = this->lastDistanceDelta * cos(this->servoTarget);
    double deltaY = this->lastDistanceDelta * sin(this->servoTarget);
    this->currentCoordinates.X += deltaX;
    this->currentCoordinates.Y += deltaY;
  }

  void drive(double velocity, double distanceDelta=0){
    this->distanceTargetSet = distanceDelta != 0;

    if(this->distanceTargetSet){
      this->lastDistanceTargetDelta = distanceDelta;
      this->distanceTarget += distanceDelta;
      this->lastMinimumDistanceError = 1000000.0;
    }

    this->velocityTarget = velocity;
    this->isFresh_ = false;
  }

  XYCoordinates deadReconingGetCurrentCoordinates(){
    return this->currentCoordinates;
  }

  double moveProgress(){
    if(!this->wheelReady()){
      double startingDistance = this->distanceTarget - this->lastDistanceTargetDelta;
      double moveRawProgress = this->lastDistance - startingDistance;
      return moveRawProgress/this->lastDistanceTargetDelta;
    }else if(!this->servoReady()){
      double servoStartingMicros = this->servoReadyAfterMicros - SERVO_FULL_ROTATION_UPDATE_SPEED*1000000.0;
      double servoRawProgress = micros() - servoStartingMicros;
      return servoRawProgress/(SERVO_FULL_ROTATION_UPDATE_SPEED*1000000.0);
    }else{
      return 1.0;
    }
  }

  void drive(MotorMove move){
    if(move.angleSet){
      this->writeServo(move.angle);
    }
    if(move.distanceSet){
      this->drive(move.velocity, move.distanceDelta);
    }
    this->lastCommandTimestampMicros = micros();
  }

  bool servoReady(){
    return this->servoReady_;
  }

  bool wheelReady(){
    return !this->distanceTargetSet;
  }

  bool ready(){
    return this->wheelReady() && this->servoReady() && this->minCommandTimePassed();
  }

  bool minCommandTimePassed(){
    return this->lastCommandTimestampMicros + MIN_COMMAND_MICROS < micros();
  }

  void writeServo(double angle){
    this->servoTarget = angle+HALF_PI;
    this->servoReady_ = this->servoTarget == this->lastServoTarget;
  }

  int readServo(){
    return this->lastServoTarget;
  }

  double currentDistance(){
    return this->distance;
  }

  double currentDistanceError(){
    return this->distancePID.getError();
  }

  double currentDistanceSteering(){
    if(this->distanceTargetSet){
      return this->distanceSteering;
    }else{
      return 1.0;
    }
  }

  double currentVelocity(){
    return this->velocity;
  }

  double currentVelocityError(){
    return this->velocityPID.getError();
  }

  double currentVelocitySteering(){
    return this->velocitySteering;
  }

  double currentSteering(){
    return this->steering;
  }

  bool isFresh(){
    return this->isFresh_;
  }

  void resetDistanceVelocity(){
    this->lastDistance = distance;
    this->distanceTarget = distance;
    this->velocity = 0;
    this->velocityTarget = this->velocity;
    this->distancePID.reset();
    this->velocityPID.reset();
    this->isFresh_ = true;
    this->distanceTargetSet = false;
  }
};

#endif