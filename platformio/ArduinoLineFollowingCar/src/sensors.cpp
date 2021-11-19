#include "sensors.h"
#include <Arduino.h>

#include "line_follow_robot_consts.h"

LineSensor::LineSensor(int analog_pin, int threshold){
    analog_pin_ = analog_pin;
    threshold_ = threshold;
}

bool LineSensor::IsOnLine(){
    return analogRead(analog_pin_) > this->threshold_;
}