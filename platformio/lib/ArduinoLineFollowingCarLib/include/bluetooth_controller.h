#ifndef ARDUINO_RESCUE_CAR_BLUETOOTH_CONTROLLER_H
#define ARDUINO_RESCUE_CAR_BLUETOOTH_CONTROLLER_H

#include <Arduino.h>
#include "line_follow_robot_consts.h"

class CarBluetoothController {

  public:
    CarBluetoothController(HardwareSerial& serial);
    void Setup();
    void Loop();
    CommandMode GetMode();
    Side GetSide();
    int GetSpeed();
    int GetPolarAngle();

  protected:
    HardwareSerial& serial_;
    CommandMode mode_ = CommandMode::kStop;
    Direction direction_ = Direction::kLeft;
    int speed_ = 0;
    int polar_angle_ = 90;
    String cmd_queue_;
    bool start_reading_ = false;

    void ParseCmdSeq(const String& seq);

};

#endif