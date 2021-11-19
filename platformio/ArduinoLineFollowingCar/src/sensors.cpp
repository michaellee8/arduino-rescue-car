#include "sensors.h"
#include <Arduino.h>

#include "line_follow_robot_consts.h"

LineSensor::LineSensor(int analog_pin, int threshold){
    analog_pin_ = analog_pin;
    threshold_ = threshold;
}

bool LineSensor::IsOnLine(){
    prev_value_ = analogRead(analog_pin_);
    return prev_value_ > threshold_;
}

int LineSensor::PrevAnalogValue(){
    return prev_value_;
}