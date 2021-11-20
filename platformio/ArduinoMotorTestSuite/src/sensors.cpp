#include "sensors.h"
#include <Arduino.h>

#include "line_follow_robot_consts.h"

LineSensor::LineSensor(int analog_pin){
    this->analog_pin = analog_pin;
}

bool LineSensor::IsOnLine(){
    return analogRead(analog_pin) <= LINE_SENSOR_THRESHOLD;
}

DigitalLineSensor::DigitalLineSensor(int digital_pin){
    this->digital_pin = digital_pin;
}

bool DigitalLineSensor::IsOnLine(){
    return digitalRead(digital_pin) == LOW;
}