#ifndef CAR_MOTORS_H
#define CAR_MOTORS_H

#include <Arduino.h>

#include "cached_motor.h"

class CarMotors {
 protected:
  CachedMotor motors_[4];
  CachedMotor& lf = motors_[0];
  CachedMotor& rf = motors_[1];
  CachedMotor& lb = motors_[2];
  CachedMotor& rb = motors_[3];

 public:
  CarMotors(CachedMotor motorA, CachedMotor motorB, CachedMotor motorC,
            CachedMotor motorD);

  // +ve for forward, -ve for backward
  void Forward(int speed);

  // +ve for right (clockwise), -ve for left (anti-clockwise)
  void Rotate(int speed);

  // +ve for right, -ve for left
  void Tilt(int speed);

  // for angle, 0 for forward, +ve for clockwise, -ve for anti-clockwise, in
  // degree
  void Translate(int speed, int angle);

  // Uses polar coordinate system instead
  void TranslatePolar(int speed, int angle);

  void SetByLetter(int a, int b, int c, int d);

  void SetByPos(int speed_lf, int speed_lb, int speed_rf, int speed_rb);

  void Stop();

  void GetMotorsSpeed(int (&speeds)[4]);
};

#endif