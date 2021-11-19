#ifndef ARDUINO_SENSORS_H
#define ARDUINO_SENSORS_H

#include <Arduino.h>

class LineSensor {
    protected:
        int analog_pin_;
        int threshold_ = 150;
    public:
        LineSensor(int analog_pin, int threshold);
        bool IsOnLine();
};

#endif