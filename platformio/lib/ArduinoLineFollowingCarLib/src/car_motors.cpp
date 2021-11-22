#include "car_motors.h"

#include <Arduino.h>

#include "cached_motor.h"

CarMotors::CarMotors(CachedMotor motorA, CachedMotor motorB, CachedMotor motorC,
                     CachedMotor motorD)
    : motors_({motorA, motorB, motorC, motorD}) {}

void CarMotors::Forward(int speed) {
  lf.SetSpeed(speed);
  lb.SetSpeed(speed);
  rf.SetSpeed(speed);
  rb.SetSpeed(speed);
}

void CarMotors::Tilt(int speed) {
  rf.SetSpeed(-speed);
  rb.SetSpeed(speed);
  lf.SetSpeed(speed);
  lb.SetSpeed(-speed);
}

void CarMotors::Rotate(int speed) {
  rf.SetSpeed(-speed);
  rb.SetSpeed(-speed);
  lf.SetSpeed(speed);
  lb.SetSpeed(speed);
}

void CarMotors::Translate(int speed, int angle) {
  auto polar_angle = 90 - angle;
  auto polar_radian = polar_angle * DEG_TO_RAD;
  auto forward_component = speed * sin(polar_radian);
  auto right_component = speed * cos(polar_radian);
  auto lf_speed = forward_component + right_component;
  auto lb_speed = forward_component - right_component;
  auto rf_speed = forward_component - right_component;
  auto rb_speed = forward_component + right_component;
  lf.SetSpeed(round(lf_speed));
  lb.SetSpeed(round(lb_speed));
  rf.SetSpeed(round(rf_speed));
  rb.SetSpeed(round(rb_speed));
}

void CarMotors::SetByLetter(int a, int b, int c, int d) {
  motors_[0].SetSpeed(a);
  motors_[1].SetSpeed(b);
  motors_[2].SetSpeed(c);
  motors_[3].SetSpeed(d);
}

void CarMotors::SetByPos(int speed_lf, int speed_lb, int speed_rf,
                         int speed_rb) {
  lf.SetSpeed(speed_lf);
  lb.SetSpeed(speed_lb);
  rf.SetSpeed(speed_rf);
  rb.SetSpeed(speed_rb);
}

void CarMotors::Stop() {
  lf.SetSpeed(0);
  lb.SetSpeed(0);
  rf.SetSpeed(0);
  rb.SetSpeed(0);
}

void CarMotors::GetMotorsSpeed(int (&speeds)[4]){
  speeds[0] = motors_[0].GetCurrentSpeed();
  speeds[1] = motors_[1].GetCurrentSpeed();
  speeds[2] = motors_[2].GetCurrentSpeed();
  speeds[3] = motors_[3].GetCurrentSpeed();
}