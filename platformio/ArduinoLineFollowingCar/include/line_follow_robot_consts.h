#ifndef LINE_FOLLOW_ROBOT_CONSTS_H
#define LINE_FOLLOW_ROBOT_CONSTS_H

#include <Arduino.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define SPEED_FACTOR 1
#define SPEED_DIV_FACTOR 2

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

#define MOTORA_REVERSED false
#define MOTORB_REVERSED true
#define MOTORC_REVERSED false
#define MOTORD_REVERSED true

#define LINE_SENSOR_THRESHOLD 500

// Middle front line sensor
#define LINE_SENSOR_MF_ANALOG_PIN A9

// Right front line sensor
#define LINE_SENSOR_RF_ANALOG_PIN A2

// Left front line sensor
#define LINE_SENSOR_LF_ANALOG_PIN A0

#define LINE_SENSOR_LF_THRESHOLD 150
#define LINE_SENSOR_MF_THRESHOLD 150
#define LINE_SENSOR_RF_THRESHOLD 225

// Temperature sensor
#define TEMPERATURE_SENSOR_PIN A8

// Buzzer
#define BUZZER_PIN A12

// RGB light module
#define RGB_LIGHT_MODULE_RED_PIN A13
#define RGB_LIGHT_MODULE_GREEN_PIN A14
#define RGB_LIGHT_MODULE_BLUE_PIN A15

#define SERIAL_USE_CLEAR_MAGIC true
#define SERIAL_CLEAR_MAGIC_STRING "\033[2J\033[1;1H"

enum Direction {
    LEFT,
    FORWARD,
    RIGHT
};



#endif