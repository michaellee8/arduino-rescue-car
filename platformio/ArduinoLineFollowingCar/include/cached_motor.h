#ifndef CACHED_MOTOR_H

#define CACHED_MOTOR_H

#include <Arduino.h>

// Skipping offsets here since it makes no sense.

// Removed all servo code since we are not using camera here.

// We prevent using the provided 8 direction based motor marcos to achieve high
// precision.

// Let's talk about DC motors confiuration here.
// The DC motors are not configured in the way explained in the graphs.
// The actual configuration is like this:
// 1  ↓A-----B↓  -1
//     |  |  |
//     |  ↓  |
// 1  ↓C-----D↓  -1
//        |
//        V
// Motor B and D are configured in reverse, so to make them drive forward, you
// need DIRX1 = HIGH and DIRX2 = LOW, and drive backward you do DIRX1 = LOW and
// DIRX2 = HIGH. Motor A and C are configured normally, so to make them drive
// forward, you need DIRX1 = LOW and DIRX2 = HIGH, and drive backward you do
// DIRX1 = HIGH and DIRX2 = LOW.

// So now we can define some functions for setting PWM (motor speed value)
// Also perform speed scaling here

// Don't use the below two boilerplate function in your code. They are here for
// convenience. Use the set_motorX_speed wrappers provided below.



// Skip setting speed if the speed has not changed.
// We peform state caching here so we don't have to handle it in the logic code.

class CachedMotor {
 protected:
  int current_speed = 0;
  bool is_reversed = false;
  int dirx1_pin;
  int dirx2_pin;
  int pwm_pin;

 public:
  CachedMotor(int dirx1_pin, int dirx2_pin, int pwm_pin, bool reversed);
  void SetSpeed(int speed);
  int GetCurrentSpeed();

 private:
  static void _set_normal_motor_speed(int dirx1_pin, int dirx2_pin, int pwm_pin,
                                      int speed);
  static void _set_reverse_motor_speed(int dirx1_pin, int dirx2_pin,
                                       int pwm_pin, int speed);
};

#endif