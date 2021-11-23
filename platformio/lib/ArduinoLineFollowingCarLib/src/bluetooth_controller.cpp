#include "bluetooth_controller.h"

#include <Arduino.h>

#include "line_follow_robot_consts.h"

CarBluetoothController::CarBluetoothController(HardwareSerial& serial)
    : serial_(serial) {}

void CarBluetoothController::Setup() { serial_.begin(38400); }

void CarBluetoothController::Loop() {
  while (serial_.available()) {
    char inputChar = serial_.read();
    if (inputChar == '(') {
      start_reading_ = true;
      continue;
    }
    if (inputChar == ')') {
      start_reading_ = false;
      // whole command sequence read, set values
      ParseCmdSeq(cmd_queue_);
      cmd_queue_ = "";
      continue;
    }
    cmd_queue_ += inputChar;
  }
}

void CarBluetoothController::ParseCmdSeq(const String& seq) {
  if (seq.charAt(0) == '0') {
    // Stop mode
    mode_ = CommandMode::kStop;
    return;
  }
  if (seq.charAt(0) >= '1' && seq.charAt(0) <= '4') {
    // Side modes
    switch (seq.charAt(0)) {
      case '1':
        mode_ = CommandMode::kYLineFollower;
        break;
      case '2':
        mode_ = CommandMode::kTGridFollowerCmdSeq;
        break;
      case '3':
        mode_ = CommandMode::kTGridFollowerCmdSingleButton;
        break;
      case '4':
        mode_ = CommandMode::kTGridFollowerCmdSinglePose;
        break;
      default:
        Serial.println("impossible 1");
        break;
    }
    switch (seq.charAt(2)) {
      case '0':
        direction_ = Direction::kLeft;
        break;
      case '1':
        direction_ = Direction::kForward;
        break;
      case '2':
        direction_ = Direction::kRight;
        break;
      default:
        Serial.println("impossible 2");
        break;
    }
  }

  if (seq.charAt(0) == '5') {
    // Manual mode
    mode_ = CommandMode::kManual;
    switch (seq.charAt(2)) {
      case '0':
        direction_ = Direction::kForward;
        break;
      case '1':
        direction_ = Direction::kRight;
        break;
      case '2':
        direction_ = Direction::kLeft;
        break;
      default:
        Serial.println("impossible 4");
        break;
    }

    auto first_comma_index = 3;
    auto second_comma_index = seq.indexOf(',', first_comma_index + 1);
  }

  Serial.println("impossible 3");
}