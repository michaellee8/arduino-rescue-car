#ifndef LINE_FOLLOW_ROBOT_CONSTS_H
#define LINE_FOLLOW_ROBOT_CONSTS_H

#include <Arduino.h>

#define kScreenWidth 128  // OLED display width, in pixels
#define kScreenHeight 32  // OLED display height, in pixels

#define kSpeedFactor 1
#define kSpeedDivFactor 1

#define kSampleSize 5

#define kRIghtSonicEchoPin A10
#define kRightSonicTrigPin A11

#define kLeftSonicEchoPin A6
#define kLeftSonicTrigPin A7

#define kSerialBaudRate 115200

#define kPwmAPin 12  // Motor A PWM
#define kDirA1Pin 34
#define kDirA2Pin 35  // Motor A Direction
#define kPwmBPin 8    // Motor B PWM
#define kDirB1Pin 37
#define kDirB2Pin 36  // Motor B Direction
#define kPwmCPin 9    // Motor C PWM
#define kDirC1Pin 43
#define kDirC2Pin 42  // Motor C Direction
#define kPwmDPin 5    // Motor D PWM
#define kDirD1Pin A4  // 26
#define kDirD2Pin A5  // 27  //Motor D Direction

#define kMotorAReversed false
#define kMotorBReversed true
#define kMotorCReversed false
#define kMotorDReversed true

#define kLineSensorThreshold 500

// Middle front line sensor
#define kLineSensorMFAnalogPin A9

// Right front line sensor
#define kLineSensorRFAnalogPin A2

// Left front line sensor
#define kLineSensorLFAnalogPin A0

#define kLineSensorLFThreshold 350
#define kLineSensorMFThreshold 350
#define kLineSensorRFThreshold 350

// Temperature sensor
#define kTemperatureSensorPin A8

// Buzzer
#define kBuzzerPin A12

// RGB light module
#define kRgbLightModuleRedPin A13
#define kRgbLightModuleGreenPin A14
#define kRgbLightModuleBluePin A15

#define kSerialUseClearMagic true
#define kSerialClearMagicString "\033[2J\033[1;1H"

#define kForwardSpeed 35
#define kRotationSpeed 25

enum class Direction {
    kLeft,
    kForward,
    kRight
};

enum class Side {
    kLeft,
    kMiddle,
    kRight
};

#endif