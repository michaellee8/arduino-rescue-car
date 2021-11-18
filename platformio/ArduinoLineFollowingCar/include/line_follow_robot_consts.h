#ifndef LINE_FOLLOW_ROBOT_CONSTS_H
#define LINE_FOLLOW_ROBOT_CONSTS_H

#include <Arduino.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define SPEED_FACTOR 1
#define SPEED_DIV_FACTOR 1

#define SAMPLE_SIZE 5

#define RIGHT_SONIC_ECHO_PIN A10
#define RIGHT_SONIC_TRIG_PIN A11

#define LEFT_SONIC_ECHO_PIN A6
#define LEFT_SONIC_TRIG_PIN A7

#define SERIAL_BAUD_RATE 115200

#define PWMA 12  // Motor A PWM
#define DIRA1 34
#define DIRA2 35  // Motor A Direction
#define PWMB 8    // Motor B PWM
#define DIRB1 37
#define DIRB2 36  // Motor B Direction
#define PWMC 9    // Motor C PWM
#define DIRC1 43
#define DIRC2 42  // Motor C Direction
#define PWMD 5    // Motor D PWM
#define DIRD1 A4  // 26
#define DIRD2 A5  // 27  //Motor D Direction

#endif