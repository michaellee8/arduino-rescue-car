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

TimedState turning_state(3000);

State gotL, gotM, gotR;

// A number variable used for debugging that will be printed on log.
// Here we use it to log application state;
// If it is negative number that means something that does not make
// sense is happening. Positive number means currently which if branch
// are we running.
int debug_number = 0;

CachedMotor motorA(DIRA1, DIRA2, PWMA, false);
CachedMotor motorB(DIRB1, DIRB2, PWMB, true);
CachedMotor motorC(DIRC1, DIRC2, PWMC, false);
CachedMotor motorD(DIRD1, DIRD2, PWMD, true);

auto motorLF = motorA;
auto motorRF = motorB;
auto motorLB = motorC;
auto motorRB = motorD;

LineSensor lineSensorLF(LINE_SENSOR_LF_ANALOG_PIN);
LineSensor lineSensorMF(LINE_SENSOR_MF_ANALOG_PIN);
LineSensor lineSensorRF(LINE_SENSOR_RF_ANALOG_PIN);

bool isLBlack;
bool isMBlack;
bool isRBlack;

void measure_sensor() {
  isLBlack = lineSensorLF.IsOnLine();
  isMBlack = lineSensorMF.IsOnLine();
  isRBlack = lineSensorRF.IsOnLine();
}

void measure_distance() {
  // Measure distance using left and right sonar here.
  // Measure the distance for SAMPLE_SIZE times, and then take the
  // average of the middle 3 samples as the final value.
  // So that we can achieve best accuracy. No need to worry about
  // time consumed in measuring here since ultrasonic measuring is quick cheap.

  float durationL;
  float durationR;
  unsigned long durationLs[SAMPLE_SIZE];
  unsigned long durationRs[SAMPLE_SIZE];

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse.

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    digitalWrite(LEFT_SONIC_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(LEFT_SONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(LEFT_SONIC_TRIG_PIN, LOW);
    durationLs[i] = pulseIn(LEFT_SONIC_ECHO_PIN, HIGH);

    // Take some rest before we measure the right sensor.
    delayMicroseconds(20);

    digitalWrite(RIGHT_SONIC_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(RIGHT_SONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(RIGHT_SONIC_TRIG_PIN, LOW);
    durationRs[i] = pulseIn(RIGHT_SONIC_ECHO_PIN, HIGH);
  }

  sortArray(durationLs, SAMPLE_SIZE);
  sortArray(durationRs, SAMPLE_SIZE);

  durationL = durationLs[SAMPLE_SIZE / 2 - 1] + durationLs[SAMPLE_SIZE / 2] +
              durationLs[SAMPLE_SIZE / 2 + 1];
  durationL /= 3.0;

  durationR = durationRs[SAMPLE_SIZE / 2 - 1] + durationRs[SAMPLE_SIZE / 2] +
              durationRs[SAMPLE_SIZE / 2 + 1];
  durationR /= 3.0;

  distance_in_cm_L = durationL / 2.0 / 29.1;
  distance_in_cm_R = durationR / 2.0 / 29.1;
}

// Log all variables to display since we don't have serial port.
void log_to_display() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.print("L");
  display.print(distance_in_cm_L, 1);
  display.print(",R");
  display.print(distance_in_cm_R, 1);
  display.print(",rb");
  display.print(motorRB.GetCurrentSpeed());
  display.print(",lb");
  display.print(motorLB.GetCurrentSpeed());
  display.print(",rf");
  display.print(motorRF.GetCurrentSpeed());
  display.print(",lf");
  display.print(motorLF.GetCurrentSpeed());
  display.print(",C");
  display.print(debug_number);
  display.display();
}

// Log all variables to Serial
void log_to_serial() {
  Serial.print("L");
  Serial.print(distance_in_cm_L, 1);
  Serial.print(",R");
  Serial.print(distance_in_cm_R, 1);
  Serial.print(",rb");
  Serial.print(motorRB.GetCurrentSpeed());
  Serial.print(",lb");
  Serial.print(motorLB.GetCurrentSpeed());
  Serial.print(",rf");
  Serial.print(motorRF.GetCurrentSpeed());
  Serial.print(",lf");
  Serial.print(motorLF.GetCurrentSpeed());
  Serial.print(",C");
  Serial.print(debug_number);
  Serial.println();
}

// Helper functions for tilting since the pattern is hard to remember.
void tilt_right(int speed) {
  motorRF.SetSpeed(-speed);
  motorRB.SetSpeed(speed);
  motorLF.SetSpeed(speed);
  motorLB.SetSpeed(-speed);
}

// tilt_left is just reverse of tilt_right. Make use of negative speed
// here for cleaner code.
void tilt_left(int speed) { tilt_right(-speed); }

void move_forward(int speed) {
  motorLB.SetSpeed(speed);
  motorLF.SetSpeed(speed);
  motorRB.SetSpeed(speed);
  motorRF.SetSpeed(speed);
}

void motor(int lf, int lb, int rf, int rb) {
  motorLF.SetSpeed(lf);
  motorLB.SetSpeed(lb);
  motorRF.SetSpeed(rf);
  motorRB.SetSpeed(rb);
}

void run_logic() {
  const Direction intended_direction = LEFT;

  if (force_forward_state.IsInside()) {
    move_forward(60);
    return;
  }

  if (turning_state.IsInside()) {
    if (intended_direction == FORWARD) {
      if (isRBlack) {
        motor(30, 30, -30, -30);
        return;
      } else {
        if (isMBlack) {
          move_forward(60);
          return;
        } else {
          move_forward(-60);
          return;
        }
      }
    } else if (intended_direction == LEFT) {
      if (isRBlack) {
        gotR.Enter();
      }
      if (gotR.IsInside() && !isLBlack && isMBlack) {
        gotM.Enter();
      }
      if (gotM.IsInside() && gotR.IsInside()) {
        turning_state.Exit();
        gotM.Exit();
        gotR.Exit();
        force_forward_state.Enter();
        return;
      } else {
        motor(-30, -30, 30, 30);
        return;
      }
    }
  }

  if (isMBlack && isLBlack) {
    turning_state.Enter();
    return;
  }

  if (isLBlack) {
    motor(-30, -30, 30, 30);
    return;
  }

  if (isRBlack) {
    motor(30, 30, -30, -30);
    return;
  }

  if (isMBlack) {
    move_forward(60);
    return;
  }

  move_forward(-60);
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
  pinMode(LINE_SENSOR_LF_DIGITAL_PIN, INPUT);
  pinMode(LINE_SENSOR_MF_ANALOG_PIN, INPUT);
  pinMode(LINE_SENSOR_MF_DIGITAL_PIN, INPUT);
  pinMode(LINE_SENSOR_RF_ANALOG_PIN, INPUT);
  pinMode(LINE_SENSOR_RF_DIGITAL_PIN, INPUT);

  // Wait for 5 seconds before starting
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Init... 5 secs");
  display.display();
  delay(5000);
}

void loop() {
  measure_distance();
  measure_sensor();
  run_logic();
  log_to_display();
  log_to_serial();
}
