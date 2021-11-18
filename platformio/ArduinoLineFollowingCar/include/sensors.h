#ifndef ARDUINO_SENSORS_H
#define ARDUINO_SENSORS_H

#include <Arduino.h>

class LineSensor {
    protected:
        int analog_pin;
    public:
        LineSensor(int analog_pin);
        bool IsOnLine();
};

#endif