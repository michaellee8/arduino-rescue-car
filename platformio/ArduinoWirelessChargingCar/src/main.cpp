#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define SPEED_FACTOR 1
#define SPEED_DIV_FACTOR 1

#define TILT_FOR_CHARGER false

#define SAMPLE_SIZE 5

#define LEFT_SONIC_ECHO_PIN A10
#define LEFT_SONIC_TRIG_PIN A11

#define RIGHT_SONIC_ECHO_PIN A6
#define RIGHT_SONIC_TRIG_PIN A7

#define SERIAL_BAUD_RATE 115200

#define PHARSE0_DISTANCE 115.0
#define PHARSE1_DISTANCE 50.0
#define PHARSE2_DISTANCE 15.0
#define PHARSE3_DISTANCE 5.0
#define PHARSE4_DISTANCE 3.5

// ArduinoSort
// Simple Insertion sort suitable for running in low memory environment like
// Arduino grabbed from GitHub.
// Just ignore it and skip to line 108.

#ifndef ArduinoSort_h
#define ArduinoSort_h

/**** These are the functions you can use ****/

// Sort an array
template <typename AnyType>
void sortArray(AnyType array[], size_t sizeOfArray);

// Sort in reverse
template <typename AnyType>
void sortArrayReverse(AnyType array[], size_t sizeOfArray);

// Sort an array with custom comparison function
template <typename AnyType>
void sortArray(AnyType array[], size_t sizeOfArray,
               bool (*largerThan)(AnyType, AnyType));

// Sort in reverse with custom comparison function
template <typename AnyType>
void sortArrayReverse(AnyType array[], size_t sizeOfArray,
                      bool (*largerThan)(AnyType, AnyType));

/**** Implementation below. Do not use below functions ****/

namespace ArduinoSort {
template <typename AnyType>
bool builtinLargerThan(AnyType first, AnyType second) {
  return first > second;
}

template <>
bool builtinLargerThan(char *first, char *second) {
  return strcmp(first, second) > 0;
}

template <typename AnyType>
void insertionSort(AnyType array[], size_t sizeOfArray, bool reverse,
                   bool (*largerThan)(AnyType, AnyType)) {
  for (size_t i = 1; i < sizeOfArray; i++) {
    for (size_t j = i; j > 0 && (largerThan(array[j - 1], array[j]) != reverse);
         j--) {
      AnyType tmp = array[j - 1];
      array[j - 1] = array[j];
      array[j] = tmp;
    }
  }
}
}  // namespace ArduinoSort

template <typename AnyType>
void sortArray(AnyType array[], size_t sizeOfArray) {
  ArduinoSort::insertionSort(array, sizeOfArray, false,
                             ArduinoSort::builtinLargerThan);
}

template <typename AnyType>
void sortArrayReverse(AnyType array[], size_t sizeOfArray) {
  ArduinoSort::insertionSort(array, sizeOfArray, true,
                             ArduinoSort::builtinLargerThan);
}

template <typename AnyType>
void sortArray(AnyType array[], size_t sizeOfArray,
               bool (*largerThan)(AnyType, AnyType)) {
  ArduinoSort::insertionSort(array, sizeOfArray, false, largerThan);
}

template <typename AnyType>
void sortArrayReverse(AnyType array[], size_t sizeOfArray,
                      bool (*largerThan)(AnyType, AnyType)) {
  ArduinoSort::insertionSort(array, sizeOfArray, true, largerThan);
}

#endif

// ArduinoSort end

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 28  // 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Arduino has no double, double is float
// float is slow anyway since it is emulated, Arduino has no FPU

// Global sensor readings are here;

float distance_in_cm_L;
float distance_in_cm_R;

float current_voltage;

// A number variable used for debugging that will be printed on log.
// Here we use it to log application state;
// If it is negative number that means something that does not make
// sense is happening. Positive number means currently which if branch
// are we running.
int debug_number = 0;

#define PWMA 12  // Motor A PWM
#define DIRA1 34
#define DIRA2 35  // Motor A Direction
#define PWMB 8    // Motor B PWM
#define DIRB1 37
#define DIRB2 36  // Motor B Direction
#define PWMC 9    // Motor C PWM
#define DIRC1 43
#define DIRC2 42  // Motor C Direction
#define PWMD 5    // Motor D PWM
#define DIRD1 A4  // 26
#define DIRD2 A5  // 27  //Motor D Direction

// Skipping offsets here since it makes no sense.

// Removed all servo code since we are not using camera here.

// We prevent using the provided 8 direction based motor marcos to achieve high
// precision.

// Let's talk about DC motors confiuration here.
// The DC motors are not configured in the way explained in the graphs.
// The actual configuration is like this:
// 1  ↓A-----B↓  -1
//     |  |  |
//     |  ↓  |
// 1  ↓C-----D↓  -1
//        |
//        V
// Motor B and D are configured in reverse, so to make them drive forward, you
// need DIRX1 = HIGH and DIRX2 = LOW, and drive backward you do DIRX1 = LOW and
// DIRX2 = HIGH. Motor A and C are configured normally, so to make them drive
// forward, you need DIRX1 = LOW and DIRX2 = HIGH, and drive backward you do
// DIRX1 = HIGH and DIRX2 = LOW.

// So now we can define some functions for setting PWM (motor speed value)
// Also perform speed scaling here

// Don't use the below two boilerplate function in your code. They are here for
// convenience. Use the set_motorX_speed wrappers provided below.

// Don't use this boilerplate function.
void _set_normal_motor_speed(int dirx1_pin, int dirx2_pin, int pwm_pin,
                             int speed) {
  int scaled_speed = speed * SPEED_FACTOR / SPEED_DIV_FACTOR;
  if (scaled_speed == 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, 0);
  } else if (scaled_speed > 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, HIGH);
    analogWrite(pwm_pin, scaled_speed);
  } else if (scaled_speed < 0) {
    digitalWrite(dirx1_pin, HIGH);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, -scaled_speed);
  }
}

// Don't use this boilerplate function.
void _set_reverse_motor_speed(int dirx1_pin, int dirx2_pin, int pwm_pin,
                              int speed) {
  int scaled_speed = speed * SPEED_FACTOR / SPEED_DIV_FACTOR;
  if (scaled_speed == 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, 0);
  } else if (scaled_speed > 0) {
    digitalWrite(dirx1_pin, HIGH);
    digitalWrite(dirx2_pin, LOW);
    analogWrite(pwm_pin, scaled_speed);
  } else if (scaled_speed < 0) {
    digitalWrite(dirx1_pin, LOW);
    digitalWrite(dirx2_pin, HIGH);
    analogWrite(pwm_pin, -scaled_speed);
  }
}

// Skip setting speed if the speed has not changed.
// We peform state caching here so we don't have to handle it in the logic code.

int motorA_current_speed;
int motorB_current_speed;
int motorC_current_speed;
int motorD_current_speed;

void set_motorA_speed(int speed) {
  if (motorA_current_speed == speed) {
    return;
  }
  _set_normal_motor_speed(DIRA1, DIRA2, PWMA, speed);
  motorA_current_speed = speed;
}

void set_motorB_speed(int speed) {
  if (motorB_current_speed == speed) {
    return;
  }
  _set_reverse_motor_speed(DIRB1, DIRB2, PWMB, speed);
  motorB_current_speed = speed;
}

void set_motorC_speed(int speed) {
  if (motorC_current_speed == speed) {
    return;
  }
  _set_normal_motor_speed(DIRC1, DIRC2, PWMC, speed);
  motorC_current_speed = speed;
}

void set_motorD_speed(int speed) {
  if (motorD_current_speed == speed) {
    return;
  }
  _set_normal_motor_speed(DIRD1, DIRD2, PWMD, speed);
  motorD_current_speed = speed;
}

// Alias of set_motorX_speed for convenience
// L for left, R for right, F for forward, B for backward.
// Use inline for one less function call, hence better performance.

inline void setLF(int speed) { set_motorD_speed(speed); }

inline void setRF(int speed) { set_motorC_speed(speed); }

inline void setLB(int speed) { set_motorB_speed(speed); }

inline void setRB(int speed) { set_motorA_speed(speed); }

void check_voltage() {
  // Subroutine for reading voltage
  int sensorValue = analogRead(A0);
  current_voltage = sensorValue * 25.0 / 1023.0;
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
    durationLs[i] = pulseIn(RIGHT_SONIC_ECHO_PIN, HIGH);

    // Take some rest before we measure the right sensor.
    delayMicroseconds(20);

    digitalWrite(RIGHT_SONIC_ECHO_PIN, LOW);
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
  display.print(",V");
  display.print(current_voltage, 1);
  display.print(",rb");
  display.print(motorA_current_speed);
  display.print(",lb");
  display.print(motorB_current_speed);
  display.print(",rf");
  display.print(motorC_current_speed);
  display.print(",lf");
  display.print(motorD_current_speed);
  display.print(",C");
  display.print(debug_number);
  display.display();
}

// Helper functions for tilting since the pattern is hard to remember.
void tilt_right(int speed){
  setRF(-speed);
  setRB(speed);
  setLF(speed);
  setLB(-speed);
}

// tilt_left is just reverse of tilt_right. Make use of negative speed
// here for cleaner code.
void tilt_left(int speed){
  tilt_right(-speed);
}

void run_motor_logic() {
  if (distance_in_cm_L > PHARSE0_DISTANCE &&
      distance_in_cm_R > PHARSE0_DISTANCE) {
    // Impossible case, probably sensor too close or misplaced.
    // We will do nothing.
    debug_number = -1;
    return;
  }
  if (distance_in_cm_R <= PHARSE0_DISTANCE &&
      distance_in_cm_L > PHARSE0_DISTANCE) {
    // Right sensor is aligned but left sensor if not aligned.
    // Tilt right slowly to get both aligned.
    debug_number = 1;
    tilt_right(40);

  }
}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}