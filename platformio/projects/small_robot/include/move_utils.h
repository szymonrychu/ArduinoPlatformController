#ifndef MOVE_UTILS_H
#define MOVE_UTILS_H

#include <Arduino.h>
#include <ServoWheel.h>
#include "config.h"

Move zeroAngles(double angle=0){
  MotorMove m;
  m.angle = angle;
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

#endif