#ifndef ARDUINO_TIMED_STATE_H

#define ARDUINO_TIMED_STATE_H

#include <Arduino.h>

class State {
    protected:
        bool is_entered_ = false;
    public:
        virtual void Enter();
        virtual void Exit();
        virtual bool IsInside();
};

class TimedState: public State {
    protected:
        unsigned long timestamp_ = 0;
        unsigned long period_ = 100;
    public:
        TimedState(unsigned long period);
        void Enter() override;
        void Exit() override;
        bool IsInside() override;
};

#endif