#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <TimedState.h>
#include <Wire.h>
#include <line_following_car_lib.h>

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 28  // 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, OLED_RESET);

// Arduino has no double, double is float
// float is slow anyway since it is emulated, Arduino has no FPU

// Global sensor readings are here;

float distance_in_cm_L;
float distance_in_cm_R;

TimedState y_turning_force_forward_state(500);

TimedState t_intersection_force_forward_state(1000);

TimedState y_intersection_left_turning_state(3000);

TimedState y_intersection_right_turning_state(3000);

TimedState last_seen_left_state(200);

TimedState last_seen_middle_state(200);

TimedState last_seen_right_state(200);

TimedState nothing_seen_continue_orignial_direciton_state(1000);

TimedState nothing_seen_back_state(2000);

TimedState t_intersection_turning_state(3000);

RepeatingTimedState logging_mask_state(1, 99, false);

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

CarMotors motors(motor_a, motor_b, motor_c, motor_d);

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
  int motor_speeds[4];
  motors.GetMotorsSpeed(motor_speeds);
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
  display.print(motor_speeds[3]);
  display.print(",lb");
  display.print(motor_speeds[2]);
  display.print(",rf");
  display.print(motor_speeds[1]);
  display.print(",lf");
  display.print(motor_speeds[0]);
  display.print(",C");
  display.print(debug_number);
  display.print(",lss");
  display.print(ConvertSideToString(last_seen_side));
  display.display();
}

// Log all variables to Serial
void LogToSerial() {
  int motor_speeds[4];
  motors.GetMotorsSpeed(motor_speeds);
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
  Serial.print(motor_speeds[3]);
  Serial.print(",lb");
  Serial.print(motor_speeds[2]);
  Serial.print(",rf");
  Serial.print(motor_speeds[1]);
  Serial.print(",lf");
  Serial.print(motor_speeds[0]);
  Serial.print(",C");
  Serial.print(debug_number);
  Serial.print(",lss");
  Serial.print(ConvertSideToString(last_seen_side));
  Serial.println();
}

void LogDebugNumberSeldomly() {
  if (logging_mask_state.isInside()) {
    Serial.println(debug_number);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(debug_number);
    display.display();
  }
}

void RunLogic() {
  const Direction intended_direction = Direction::kLeft;

  if (y_turning_force_forward_state.isInside()) {
    motors.Forward(kForwardSpeed);
    debug_number = 11;
    return;
  }

  if (y_intersection_left_turning_state.isInside()) {
    if (intended_direction == Direction::kForward ||
        intended_direction == Direction::kRight) {
      if (is_r_black) {
        motors.Rotate(kRotationSpeed);
        debug_number = 12;
        return;
      } else {
        if (is_m_black) {
          motors.Forward(kForwardSpeed);
          debug_number = 13;
          return;
        } else {
          motors.Forward(kForwardSpeed);
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
        y_turning_force_forward_state.enter();
        debug_number = 15;
        return;
      } else {
        motors.Rotate(-kRotationSpeed);
        debug_number = 16;
        return;
      }
    }
  }

  if (y_intersection_right_turning_state.isInside()) {
    if (intended_direction == Direction::kForward ||
        intended_direction == Direction::kLeft) {
      if (is_r_black) {
        motors.Rotate(kRotationSpeed);
        debug_number = 12;
        return;
      } else {
        if (is_m_black) {
          motors.Forward(kForwardSpeed);
          debug_number = 13;
          return;
        } else {
          motors.Forward(kForwardSpeed);
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
        y_turning_force_forward_state.enter();
        debug_number = 29;
        return;
      } else {
        motors.Rotate(kRotationSpeed);
        debug_number = 30;
        return;
      }
    }
  }

  if ((is_l_black && is_m_black && is_r_black) ||
      (last_seen_left_state.isInside() && last_seen_middle_state.isInside() &&
       last_seen_right_state.isInside())) {
    t_intersection_turning_state.enter();
    debug_number = 24;
    return;
  }

  if ((is_m_black && is_l_black) ||
      (last_seen_left_state.isInside() && last_seen_middle_state.isInside())) {
    y_intersection_left_turning_state.enter();
    debug_number = 17;
    return;
  }

  if ((is_m_black && is_r_black) ||
      (last_seen_middle_state.isInside() && last_seen_right_state.isInside())) {
    y_intersection_right_turning_state.enter();
    debug_number = 31;
    return;
  }

  if (is_l_black) {
    last_seen_side = Side::kLeft;
    last_seen_left_state.forceEnter();
    motors.Rotate(-kRotationSpeed);
    debug_number = 18;
    return;
  }

  if (is_r_black) {
    last_seen_side = Side::kRight;
    last_seen_right_state.forceEnter();
    motors.Rotate(kRotationSpeed);
    debug_number = 19;
    return;
  }

  if (is_m_black) {
    last_seen_side = Side::kMiddle;
    last_seen_middle_state.forceEnter();
    motors.Forward(kForwardSpeed);
    debug_number = 20;
    return;
  }

  // All sensors had no input.
  switch (last_seen_side) {
    case Side::kLeft:
      motors.Rotate(-kRotationSpeed);
      debug_number = 21;
      break;

    case Side::kRight:
      motors.Rotate(kRotationSpeed);
      debug_number = 22;
      break;

    case Side::kMiddle:
      motors.Forward(kForwardSpeed);
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
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.print("Init... 5 secs");
  display.display();

  logging_mask_state.enter();
  delay(5000);
}

void loop() {
  // MeasureDistance();
  MeasureLineSensor();
  RunLogic();
  LogDebugNumberSeldomly();
  // LogToDisplay();
  // LogToSerial();
}
