#ifndef LINE_FOLLOW_ROBOT_CONSTS_H
#define LINE_FOLLOW_ROBOT_CONSTS_H

#include <Arduino.h>

const auto kScreenWidth = 128;  // OLED display width, in pixels
const auto kScreenHeight = 32;  // OLED display height, in pixels

const auto kSpeedFactor = 1;
const auto kSpeedDivFactor = 1;

const auto kSampleSize = 5;

const auto kRIghtSonicEchoPin = A10;
const auto kRightSonicTrigPin = A11;

const auto kLeftSonicEchoPin = A6;
const auto kLeftSonicTrigPin = A7;

const auto kSerialBaudRate = 115200;

const auto kPwmAPin = 12;  // Motor A PWM
const auto kDirA1Pin = 34;
const auto kDirA2Pin = 35;  // Motor A Direction
const auto kPwmBPin = 8;    // Motor B PWM
const auto kDirB1Pin = 37;
const auto kDirB2Pin = 36;  // Motor B Direction
const auto kPwmCPin = 9;    // Motor C PWM
const auto kDirC1Pin = 43;
const auto kDirC2Pin = 42;  // Motor C Direction
const auto kPwmDPin = 5;    // Motor D PWM
const auto kDirD1Pin = A4;  // 26
const auto kDirD2Pin = A5;  // 27  //Motor D Direction

const auto kMotorAReversed = false;
const auto kMotorBReversed = true;
const auto kMotorCReversed = false;
const auto kMotorDReversed = true;

const auto kLineSensorThreshold = 350;

// Middle front line sensor
const auto kLineSensorMFAnalogPin = A9;

// Right front line sensor
const auto kLineSensorRFAnalogPin = A2;

// Left front line sensor
const auto kLineSensorLFAnalogPin = A0;

// Middle middle line sensor
const auto kLineSensorMMAnalogPin = A13;

// Right middle line sensor
const auto kLineSensorRMAnalogPin = A15;

// Left middle line sensor
const auto kLineSensorLMAnalogPin = A14;

const auto kLineSensorLFThreshold = 350;
const auto kLineSensorMFThreshold = 350;
const auto kLineSensorRFThreshold = 350;

const auto kLineSensorLMThreshold = 350;
const auto kLineSensorMMThreshold = 350;
const auto kLineSensorRMThreshold = 350;

// Temperature sensor
const auto kTemperatureSensorPin = A8;

// Buzzer
const auto kBuzzerPin = A12;

// RGB light module
const auto kRgbLightModuleRedPin = A13;
const auto kRgbLightModuleGreenPin = A14;
const auto kRgbLightModuleBluePin = A15;

const auto kSerialUseClearMagic = true;
const char kSerialClearMagicString[] = {27, 91, 50, 74, 27, 91, 49, 59, 49, 72};

const auto kForwardSpeed = 35;
const auto kRotationSpeed = 35;

enum class Direction { kLeft, kForward, kRight };

enum class Side { kLeft, kMiddle, kRight };

enum class CommandMode {
    kStop,
    kYLineFollower,
    kTGridFollowerCmdSeq,
    kTGridFollowerCmdSingleButton,
    kTGridFollowerCmdSinglePose,
    kManual
};

#endif