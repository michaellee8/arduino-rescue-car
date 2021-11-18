#include "cached_motor.h"

#include <Arduino.h>

#include "line_follow_robot_consts.h"

CachedMotor::CachedMotor(int dirx1_pin, int dirx2_pin, int pwm_pin,
                         bool reversed) {
  this->dirx1_pin = dirx1_pin;
  this->dirx2_pin = dirx2_pin;
  this->pwm_pin = pwm_pin;
  this->reveresed = reveresed;
}

void CachedMotor::SetSpeed(int speed) {
  if (reveresed) {
    _set_reverse_motor_speed(dirx1_pin, dirx2_pin, pwm_pin, speed);
  } else {
    _set_normal_motor_speed(dirx1_pin, dirx2_pin, pwm_pin, speed);
  }
}

void CachedMotor::_set_normal_motor_speed(int dirx1_pin, int dirx2_pin,
                                          int pwm_pin, int speed) {
  int scaled_speed = speed * SPEED_FACTOR / SPEED_DIV_FACTOR;
  if (scaled_speed == 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, 0);
  } else if (scaled_speed > 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, HIGH);
    analogWrite(pwm_pin, scaled_speed);
  } else if (scaled_speed < 0) {
    digitalWrite(dirx1_pin, HIGH);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, -scaled_speed);
  }
}

void CachedMotor::_set_reverse_motor_speed(int dirx1_pin, int dirx2_pin,
                                           int pwm_pin, int speed) {
  int scaled_speed = speed * SPEED_FACTOR / SPEED_DIV_FACTOR;
  if (scaled_speed == 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, 0);
  } else if (scaled_speed > 0) {
    digitalWrite(dirx1_pin, HIGH);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, scaled_speed);
  } else if (scaled_speed < 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, HIGH);
    analogWrite(pwm_pin, -scaled_speed);
  }
}