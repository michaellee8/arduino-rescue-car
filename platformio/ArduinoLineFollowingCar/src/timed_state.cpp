#include "timed_state.h"

#include <Arduino.h>

void State::enter() { isEntered = true; }

void State::exit() { isEntered = false; }

bool State::isInside() { return isEntered; }

TimedState::TimedState(unsigned long period) { this->period = period; }

void TimedState::enter() {
  if (!isInside()) {
    isEntered = true;
    timestamp = millis();
  }
}

bool TimedState::isInside() {
  return isEntered && millis() - timestamp <= period;
}

void TimedState::exit() {
  isEntered = false;
  timestamp = millis() - period - 100;
}