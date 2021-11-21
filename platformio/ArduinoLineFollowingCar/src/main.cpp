#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TimedState.h>

#include "arduino_sort.h"
#include "cached_motor.h"
#include "line_follow_robot_consts.h"
#include "sensors.h"
#include "utils.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 28  // 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, OLED_RESET);

// Arduino has no double, double is float
// float is slow anyway since it is emulated, Arduino has no FPU

// Global sensor readings are here;

float distance_in_cm_L;
float distance_in_cm_R;

TimedState force_forward_state(500);

TimedState t_intersection_force_forward_state(1000);

TimedState y_intersection_left_turning_state(3000);

TimedState y_intersection_right_turning_state(3000);

TimedState nothing_seen_continue_orignial_direciton_state(1000);

TimedState nothing_seen_back_state(2000);

TimedState t_intersection_turning_state(3000);

SimpleState got_l, got_m, got_r;

SimpleState t_intersection_cleared;

SimpleState t_got_l, t_got_m, t_got_r;

// A number variable used for debugging that will be printed on log.
// Here we use it to log application state;
// If it is negative number that means something that does not make
// sense is happening. Positive number means currently which if branch
// are we running.
int debug_number = 0;

CachedMotor motor_a(kDirA1Pin, kDirA2Pin, kPwmAPin, kMotorAReversed);
CachedMotor motor_b(kDirB1Pin, kDirB2Pin, kPwmBPin, kMotorBReversed);
CachedMotor motor_c(kDirC1Pin, kDirC2Pin, kPwmCPin, kMotorCReversed);
CachedMotor motor_d(kDirD1Pin, kDirD2Pin, kPwmDPin, kMotorDReversed);

auto motor_lf = motor_a;
auto motor_rf = motor_b;
auto motor_lb = motor_c;
auto motor_rb = motor_d;

LineSensor line_sensor_lf(kLineSensorLFAnalogPin, kLineSensorLFThreshold);
LineSensor line_sensor_mf(kLineSensorMFAnalogPin, kLineSensorMFThreshold);
LineSensor line_sensor_rf(kLineSensorRFAnalogPin, kLineSensorRFThreshold);

bool is_l_black;
bool is_m_black;
bool is_r_black;

Side last_seen_side = Side::kMiddle;

void MeasureLineSensor() {
  is_l_black = line_sensor_lf.IsOnLine();
  is_m_black = line_sensor_mf.IsOnLine();
  is_r_black = line_sensor_rf.IsOnLine();
}

void MeasureDistance() {
  // Measure distance using left and right sonar here.
  // Measure the distance for kSampleSize times, and then take the
  // average of the middle 3 samples as the final value.
  // So that we can achieve best accuracy. No need to worry about
  // time consumed in measuring here since ultrasonic measuring is quick cheap.

  float duration_l;
  float duration_r;
  unsigned long duration_l_s[kSampleSize];
  unsigned long duration_r_s[kSampleSize];

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse.

  for (int i = 0; i < kSampleSize; i++) {
    digitalWrite(kLeftSonicTrigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(kLeftSonicTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(kLeftSonicTrigPin, LOW);
    duration_l_s[i] = pulseIn(kLeftSonicEchoPin, HIGH);

    // Take some rest before we measure the right sensor.
    delayMicroseconds(20);

    digitalWrite(kRightSonicTrigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(kRightSonicTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(kRightSonicTrigPin, LOW);
    duration_r_s[i] = pulseIn(kRIghtSonicEchoPin, HIGH);
  }

  sortArray(duration_l_s, kSampleSize);
  sortArray(duration_r_s, kSampleSize);

  duration_l = duration_l_s[kSampleSize / 2 - 1] +
               duration_l_s[kSampleSize / 2] +
               duration_l_s[kSampleSize / 2 + 1];
  duration_l /= 3.0;

  duration_r = duration_r_s[kSampleSize / 2 - 1] +
               duration_r_s[kSampleSize / 2] +
               duration_r_s[kSampleSize / 2 + 1];
  duration_r /= 3.0;

  distance_in_cm_L = duration_l / 2.0 / 29.1;
  distance_in_cm_R = duration_r / 2.0 / 29.1;
}

// Log all variables to display since we don't have serial port.
void LogToDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.print("L");
  display.print(is_l_black);
  display.print(",Lv");
  display.print(line_sensor_lf.PrevAnalogValue());
  display.print(",M");
  display.print(is_m_black);
  display.print(",Mv");
  display.print(line_sensor_mf.PrevAnalogValue());
  display.print(",R");
  display.print(is_r_black);
  display.print(",Rv");
  display.print(line_sensor_rf.PrevAnalogValue());
  display.print(",rb");
  display.print(motor_rb.GetCurrentSpeed());
  display.print(",lb");
  display.print(motor_lb.GetCurrentSpeed());
  display.print(",rf");
  display.print(motor_rf.GetCurrentSpeed());
  display.print(",lf");
  display.print(motor_lf.GetCurrentSpeed());
  display.print(",C");
  display.print(debug_number);
  display.print(",lss");
  display.print(ConvertSideToString(last_seen_side));
  display.display();
}

// Log all variables to Serial
void LogToSerial() {
  Serial.print("L");
  Serial.print(is_l_black);
  Serial.print(",Lv");
  Serial.print(line_sensor_lf.PrevAnalogValue());
  Serial.print(",M");
  Serial.print(is_m_black);
  Serial.print(",Mv");
  Serial.print(line_sensor_mf.PrevAnalogValue());
  Serial.print(",R");
  Serial.print(is_r_black);
  Serial.print(",Rv");
  Serial.print(line_sensor_rf.PrevAnalogValue());
  Serial.print(",rb");
  Serial.print(motor_rb.GetCurrentSpeed());
  Serial.print(",lb");
  Serial.print(motor_lb.GetCurrentSpeed());
  Serial.print(",rf");
  Serial.print(motor_rf.GetCurrentSpeed());
  Serial.print(",lf");
  Serial.print(motor_lf.GetCurrentSpeed());
  Serial.print(",C");
  Serial.print(debug_number);
  Serial.print(",lss");
  Serial.print(ConvertSideToString(last_seen_side));
  Serial.println();
}

// Helper functions for tilting since the pattern is hard to remember.
void TiltRight(int speed) {
  motor_rf.SetSpeed(-speed);
  motor_rb.SetSpeed(speed);
  motor_lf.SetSpeed(speed);
  motor_lb.SetSpeed(-speed);
}

// TiltLeft is just reverse of TiltRight. Make use of negative speed
// here for cleaner code.
void TiltLeft(int speed) { TiltRight(-speed); }

void MoveForward(int speed) {
  motor_lb.SetSpeed(speed);
  motor_lf.SetSpeed(speed);
  motor_rb.SetSpeed(speed);
  motor_rf.SetSpeed(speed);
}

void RunMotor(int lf, int lb, int rf, int rb) {
  motor_lf.SetSpeed(lf);
  motor_lb.SetSpeed(lb);
  motor_rf.SetSpeed(rf);
  motor_rb.SetSpeed(rb);
}

void RunLogic() {
  const Direction intended_direction = Direction::kRight;

  if (force_forward_state.isInside() ||
      t_intersection_force_forward_state.isInside()) {
    MoveForward(kForwardSpeed);
    debug_number = 11;
    return;
  }

  if (t_intersection_turning_state.isInside()) {
    if (is_l_black || is_r_black) {
      // We are not clear from the intersection yet.
      debug_number = 25;
      MoveForward(kForwardSpeed);
      return;
    }
    if (is_m_black) {
      debug_number = 26;
      t_intersection_cleared.enter();
      t_intersection_force_forward_state.enter();
      return;
    }
    if (!t_intersection_cleared.isInside()) {
      // Impossible case!
      debug_number = 27;
      RunMotor(0, 0, 0, 0);
      return;
    }
    // We are cleared! Let's do the rotation.
    if (intended_direction == Direction::kForward) {
      debug_number = 28;
      // To move forward, just exit the state;
      t_intersection_turning_state.exit();
      t_intersection_cleared.exit();
      t_got_l.exit();
      t_got_m.exit();
      t_got_r.exit();
      return;
    }

    if (intended_direction == Direction::kLeft) {
      if (is_l_black) {
        t_got_l.enter();
      }
      if (t_got_l.isInside() && is_m_black) {
        t_got_m.enter();
      }
      if (t_got_l.isInside() && t_got_m.isInside()) {
        t_intersection_turning_state.exit();
        t_got_l.exit();
        t_got_m.exit();
        t_intersection_cleared.exit();
        debug_number = 29;
        return;
      } else {
        RunMotor(-kRotationSpeed, -kRotationSpeed, kRotationSpeed,
                 kRotationSpeed);
        debug_number = 30;
        return;
      }
    }

    if (intended_direction == Direction::kRight) {
      if (is_r_black) {
        t_got_r.enter();
      }
      if (t_got_r.isInside() && is_m_black) {
        t_got_m.enter();
      }
      if (t_got_r.isInside() && t_got_m.isInside()) {
        t_intersection_turning_state.exit();
        t_got_r.exit();
        t_got_m.exit();
        t_intersection_cleared.exit();
        debug_number = 31;
        return;
      } else {
        RunMotor(kRotationSpeed, kRotationSpeed, -kRotationSpeed,
                 -kRotationSpeed);
        debug_number = 32;
        return;
      }
    }
  }

  if (y_intersection_left_turning_state.isInside()) {
    if (intended_direction == Direction::kForward ||
        intended_direction == Direction::kRight) {
      if (is_r_black) {
        RunMotor(kRotationSpeed, kRotationSpeed, -kRotationSpeed,
                 -kRotationSpeed);
        debug_number = 12;
        return;
      } else {
        if (is_m_black) {
          MoveForward(kForwardSpeed);
          debug_number = 13;
          return;
        } else {
          MoveForward(kForwardSpeed);
          debug_number = 14;
          return;
        }
      }
    } else if (intended_direction == Direction::kLeft) {
      if (is_r_black) {
        got_r.enter();
      }
      if (got_r.isInside() && !is_l_black && is_m_black) {
        got_m.enter();
      }
      if (got_m.isInside() && got_r.isInside()) {
        y_intersection_left_turning_state.exit();
        got_m.exit();
        got_r.exit();
        force_forward_state.enter();
        debug_number = 15;
        return;
      } else {
        RunMotor(-kRotationSpeed, -kRotationSpeed, kRotationSpeed,
                 kRotationSpeed);
        debug_number = 16;
        return;
      }
    }
  }

  if (y_intersection_right_turning_state.isInside()) {
    if (intended_direction == Direction::kForward ||
        intended_direction == Direction::kLeft) {
      if (is_r_black) {
        RunMotor(kRotationSpeed, kRotationSpeed, -kRotationSpeed,
                 -kRotationSpeed);
        debug_number = 12;
        return;
      } else {
        if (is_m_black) {
          MoveForward(kForwardSpeed);
          debug_number = 13;
          return;
        } else {
          MoveForward(kForwardSpeed);
          debug_number = 14;
          return;
        }
      }
    } else if (intended_direction == Direction::kRight) {
      if (is_l_black) {
        got_l.enter();
      }
      if (got_l.isInside() && !is_r_black && is_m_black) {
        got_m.enter();
      }
      if (got_m.isInside() && got_r.isInside()) {
        y_intersection_left_turning_state.exit();
        got_m.exit();
        got_l.exit();
        force_forward_state.enter();
        debug_number = 29;
        return;
      } else {
        RunMotor(kRotationSpeed, kRotationSpeed, -kRotationSpeed,
                 -kRotationSpeed);
        debug_number = 30;
        return;
      }
    }
  }

  if (is_l_black && is_m_black && is_r_black) {
    t_intersection_turning_state.enter();
    debug_number = 24;
    return;
  }

  if (is_m_black && is_l_black && !t_intersection_turning_state.isInside()) {
    y_intersection_left_turning_state.enter();
    debug_number = 17;
    return;
  }

  if (is_m_black && is_r_black && !t_intersection_turning_state.isInside()) {
    y_intersection_right_turning_state.enter();
    debug_number = 31;
    return;
  }

  if (is_l_black) {
    last_seen_side = Side::kLeft;
    RunMotor(-kRotationSpeed, -kRotationSpeed, kRotationSpeed, kRotationSpeed);
    debug_number = 18;
    return;
  }

  if (is_r_black) {
    last_seen_side = Side::kRight;
    RunMotor(kRotationSpeed, kRotationSpeed, -kRotationSpeed, -kRotationSpeed);
    debug_number = 19;
    return;
  }

  if (is_m_black) {
    last_seen_side = Side::kMiddle;
    MoveForward(kForwardSpeed);
    debug_number = 20;
    return;
  }

  // All sensors had no input.
  switch (last_seen_side) {
    case Side::kLeft:
      RunMotor(-kRotationSpeed, -kRotationSpeed, kRotationSpeed,
               kRotationSpeed);
      debug_number = 21;
      break;

    case Side::kRight:
      RunMotor(kRotationSpeed, kRotationSpeed, -kRotationSpeed,
               -kRotationSpeed);
      debug_number = 22;
      break;

    case Side::kMiddle:
      MoveForward(kForwardSpeed);
      debug_number = 23;
      break;
  }
}

void setup() {
  Serial.begin(kSerialBaudRate);

  // OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.setRotation(2);

  // Setup Voltage detector
  pinMode(A0, INPUT);

  // Setup ultrasonic sensor
  pinMode(kLeftSonicEchoPin, INPUT);
  pinMode(kLeftSonicTrigPin, OUTPUT);
  pinMode(kRIghtSonicEchoPin, INPUT);
  pinMode(kRightSonicTrigPin, OUTPUT);

  // Setup line sensor
  pinMode(kLineSensorLFAnalogPin, INPUT);
  pinMode(kLineSensorMFAnalogPin, INPUT);
  pinMode(kLineSensorRFAnalogPin, INPUT);

  // Wait for 5 seconds before starting
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Init... 5 secs");
  display.display();
  delay(5000);
}

void loop() {
  // MeasureDistance();
  MeasureLineSensor();
  RunLogic();
  // LogToDisplay();
  // LogToSerial();
}
