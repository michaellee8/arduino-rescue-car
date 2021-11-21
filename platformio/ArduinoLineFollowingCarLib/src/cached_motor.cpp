#include "cached_motor.h"

#include <Arduino.h>

#include "line_follow_robot_consts.h"

CachedMotor::CachedMotor(int dirx1_pin, int dirx2_pin, int pwm_pin,
                         bool reversed) {
  dirx1_pin_ = dirx1_pin;
  dirx2_pin_ = dirx2_pin;
  pwm_pin_ = pwm_pin;
  is_reversed_ = reversed;
}

void CachedMotor::SetSpeed(int speed) {
  if (current_speed_ == speed) {
    return;
  }
  if (is_reversed_) {
    _set_reverse_motor_speed(dirx1_pin_, dirx2_pin_, pwm_pin_, speed);
  } else {
    _set_normal_motor_speed(dirx1_pin_, dirx2_pin_, pwm_pin_, speed);
  }
  current_speed_ = speed;
}

int CachedMotor::GetCurrentSpeed() { return current_speed_; }

void CachedMotor::_set_normal_motor_speed(int dirx1_pin, int dirx2_pin,
                                          int pwm_pin, int speed) {
  int scaled_speed = speed * kSpeedFactor / kSpeedDivFactor;
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
  int scaled_speed = speed * kSpeedFactor / kSpeedDivFactor;
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
