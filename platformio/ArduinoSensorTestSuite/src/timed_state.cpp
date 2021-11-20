#include "timed_state.h"

#include <Arduino.h>

void State::Enter() { isEntered = true; }

void State::Exit() { isEntered = false; }

bool State::IsInside() { return isEntered; }

TimedState::TimedState(unsigned long period) { this->period = period; }

void TimedState::Enter() {
  if (!IsInside()) {
    isEntered = true;
    timestamp = millis();
  }
}

bool TimedState::IsInside() {
  return isEntered && millis() - timestamp <= period;
}

void TimedState::Exit() {
  isEntered = false;
  timestamp = millis() - period - 100;
}