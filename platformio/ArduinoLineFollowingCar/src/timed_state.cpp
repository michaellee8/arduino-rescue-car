#include "timed_state.h"

#include <Arduino.h>

void State::Enter() { is_entered_ = true; }

void State::Exit() { is_entered_ = false; }

bool State::IsInside() { return is_entered_; }

TimedState::TimedState(unsigned long period) { period_ = period; }

void TimedState::Enter() {
  if (!IsInside()) {
    is_entered_ = true;
    timestamp_ = millis();
  }
}

bool TimedState::IsInside() {
  return is_entered_ && millis() - timestamp_ <= period_;
}

void TimedState::Exit() {
  is_entered_ = false;
  timestamp_ = millis() - period_ - 100;
}