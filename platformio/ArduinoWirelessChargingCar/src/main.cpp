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

// ArduinoSort
// Simple Insertion sort suitable for running in low memory environment like
// Arduino grabbed from GitHub

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
float distance_in_cmL;
float distance_in_cmR;

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

void set_motorA_speed(int speed) {
  _set_normal_motor_speed(DIRA1, DIRA2, PWMA, speed);
}

void set_motorB_speed(int speed) {
  _set_reverse_motor_speed(DIRB1, DIRB2, PWMB, speed);
}

void set_motorC_speed(int speed) {
  _set_normal_motor_speed(DIRC1, DIRC2, PWMC, speed);
}

void set_motorD_speed(int speed) {
  _set_reverse_motor_speed(DIRD1, DIRD2, PWMD, speed);
}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}