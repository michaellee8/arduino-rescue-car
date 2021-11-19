#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "arduino_sort.h"
#include "cached_motor.h"
#include "line_follow_robot_consts.h"
#include "sensors.h"
#include "timed_state.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 28  // 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Arduino has no double, double is float
// float is slow anyway since it is emulated, Arduino has no FPU

// Global sensor readings are here;

float distance_in_cm_L;
float distance_in_cm_R;

TimedState force_forward_state(500);

// TimedState turning_state(3000);

TimedState turning_state(0);

TimedState nothing_seen_continue_orignial_direciton_state(1000);

TimedState nothing_seen_back_state(2000);

State got_l, got_m, got_r;

// A number variable used for debugging that will be printed on log.
// Here we use it to log application state;
// If it is negative number that means something that does not make
// sense is happening. Positive number means currently which if branch
// are we running.
int debug_number = 0;

CachedMotor motor_a(DIRA1, DIRA2, PWMA, false);
CachedMotor motor_b(DIRB1, DIRB2, PWMB, true);
CachedMotor motor_c(DIRC1, DIRC2, PWMC, false);
CachedMotor motor_d(DIRD1, DIRD2, PWMD, true);

auto motor_lf = motor_a;
auto motor_rf = motor_b;
auto motor_lb = motor_c;
auto motor_rb = motor_d;

LineSensor line_sensor_lf(LINE_SENSOR_LF_ANALOG_PIN, LINE_SENSOR_LF_THRESHOLD);
LineSensor line_sensor_mf(LINE_SENSOR_MF_ANALOG_PIN, LINE_SENSOR_MF_THRESHOLD);
LineSensor line_sensor_rf(LINE_SENSOR_RF_ANALOG_PIN, LINE_SENSOR_RF_THRESHOLD);

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
  // Measure the distance for SAMPLE_SIZE times, and then take the
  // average of the middle 3 samples as the final value.
  // So that we can achieve best accuracy. No need to worry about
  // time consumed in measuring here since ultrasonic measuring is quick cheap.

  float duration_l;
  float duration_r;
  unsigned long duration_l_s[SAMPLE_SIZE];
  unsigned long duration_r_s[SAMPLE_SIZE];

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse.

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    digitalWrite(LEFT_SONIC_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(LEFT_SONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(LEFT_SONIC_TRIG_PIN, LOW);
    duration_l_s[i] = pulseIn(LEFT_SONIC_ECHO_PIN, HIGH);

    // Take some rest before we measure the right sensor.
    delayMicroseconds(20);

    digitalWrite(RIGHT_SONIC_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(RIGHT_SONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(RIGHT_SONIC_TRIG_PIN, LOW);
    duration_r_s[i] = pulseIn(RIGHT_SONIC_ECHO_PIN, HIGH);
  }

  sortArray(duration_l_s, SAMPLE_SIZE);
  sortArray(duration_r_s, SAMPLE_SIZE);

  duration_l = duration_l_s[SAMPLE_SIZE / 2 - 1] +
               duration_l_s[SAMPLE_SIZE / 2] +
               duration_l_s[SAMPLE_SIZE / 2 + 1];
  duration_l /= 3.0;

  duration_r = duration_r_s[SAMPLE_SIZE / 2 - 1] +
               duration_r_s[SAMPLE_SIZE / 2] +
               duration_r_s[SAMPLE_SIZE / 2 + 1];
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
  display.print(",M");
  display.print(is_m_black);
  display.print(",R");
  display.print(is_r_black);
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
  display.print(last_seen_side);
  display.display();
}

// Log all variables to Serial
void LogToSerial() {
  Serial.print("L");
  Serial.print(is_l_black);
  Serial.print(",M");
  Serial.print(is_m_black);
  Serial.print(",R");
  Serial.print(is_r_black);
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
  const Direction intended_direction = Direction::kLeft;

  if (force_forward_state.IsInside()) {
    MoveForward(60);
    debug_number = 11;
    return;
  }

  if (turning_state.IsInside()) {
    if (intended_direction == Direction::kForward) {
      if (is_r_black) {
        RunMotor(30, 30, -30, -30);
        debug_number = 12;
        return;
      } else {
        if (is_m_black) {
          MoveForward(60);
          debug_number = 13;
          return;
        } else {
          MoveForward(-60);
          debug_number = 14;
          return;
        }
      }
    } else if (intended_direction == Direction::kLeft) {
      if (is_r_black) {
        got_r.Enter();
      }
      if (got_r.IsInside() && !is_l_black && is_m_black) {
        got_m.Enter();
      }
      if (got_m.IsInside() && got_r.IsInside()) {
        turning_state.Exit();
        got_m.Exit();
        got_r.Exit();
        force_forward_state.Enter();
        debug_number = 15;
        return;
      } else {
        RunMotor(-30, -30, 30, 30);
        debug_number = 16;
        return;
      }
    }
  }

  if (is_m_black && is_l_black) {
    turning_state.Enter();
    debug_number = 17;
    return;
  }

  if (is_l_black) {
    last_seen_side = Side::kLeft;
    RunMotor(-30, -30, 30, 30);
    debug_number = 18;
    return;
  }

  if (is_r_black) {
    last_seen_side = Side::kRight;
    RunMotor(30, 30, -30, -30);
    debug_number = 19;
    return;
  }

  if (is_m_black) {
    last_seen_side = Side::kMiddle;
    MoveForward(30);
    debug_number = 20;
    return;
  }

  // All sensors had no input.
  switch (last_seen_side) {
    case Side::kLeft:
      RunMotor(-30, -30, 30, 30);
      debug_number = 21;
      break;

    case Side::kRight:
      RunMotor(30, 30, -30, -30);
      debug_number = 22;
      break;

    case Side::kMiddle:
      MoveForward(30);
      debug_number = 23;
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.setRotation(2);

  // Setup Voltage detector
  pinMode(A0, INPUT);

  // Setup ultrasonic sensor
  pinMode(LEFT_SONIC_ECHO_PIN, INPUT);
  pinMode(LEFT_SONIC_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_SONIC_ECHO_PIN, INPUT);
  pinMode(RIGHT_SONIC_TRIG_PIN, OUTPUT);

  // Setup line sensor
  pinMode(LINE_SENSOR_LF_ANALOG_PIN, INPUT);
  pinMode(LINE_SENSOR_MF_ANALOG_PIN, INPUT);
  pinMode(LINE_SENSOR_RF_ANALOG_PIN, INPUT);

  // Wait for 5 seconds before starting
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Init... 5 secs");
  display.display();
  delay(5000);
}

void loop() {
  MeasureDistance();
  MeasureLineSensor();
  RunLogic();
  LogToDisplay();
  LogToSerial();
}
