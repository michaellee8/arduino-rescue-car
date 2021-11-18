#ifndef ARDUINO_TIMED_STATE_H

#define ARDUINO_TIMED_STATE_H

#include <Arduino.h>

class State {
    protected:
        bool isEntered = false;
    public:
        virtual void enter();
        virtual void exit();
        virtual bool isInside();
};

class TimedState: public State {
    protected:
        unsigned long timestamp = 0;
        unsigned long period = 100;
    public:
        TimedState(unsigned long period);
        void enter() override;
        void exit() override;
        bool isInside() override;
};

#endif