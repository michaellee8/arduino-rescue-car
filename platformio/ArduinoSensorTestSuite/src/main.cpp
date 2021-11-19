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

// A number variable used for debugging that will be printed on log.
// Here we use it to log application state;
// If it is negative number that means something that does not make
// sense is happening. Positive number means currently which if branch
// are we running.
int debug_number = 0;

void run_logic() {

  const int WAIT_TIME = 10 * 1000;

  int currentTime = millis();

  int lineSensorLFAnalogValue = analogRead(LINE_SENSOR_LF_ANALOG_PIN);
  int lineSensorMFAnalogValue = analogRead(LINE_SENSOR_MF_ANALOG_PIN);
  int lineSensorRFAnalogValue = analogRead(LINE_SENSOR_RF_ANALOG_PIN);

  int temperatureSensorAnalogValue = analogRead(TEMPERATURE_SENSOR_PIN);

  int buzzerOutput = ((currentTime / WAIT_TIME) % 5 + 1) * 200;

  int rgbRedOutput = (currentTime / WAIT_TIME) % 3 == 0 ? 255 : 0;
  int rgbBlueOutput = (currentTime / WAIT_TIME) % 3 == 1 ? 255 : 0;
  int rgbGreenOutput = (currentTime / WAIT_TIME) % 3 == 2 ? 255 : 0;

  display.clearDisplay();
  display.cp437(true);
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.print("LSLFA:");
  display.print(lineSensorLFAnalogValue);
  display.print(", ");

  display.print("LSMFA:");
  display.print(lineSensorMFAnalogValue);
  display.print(", ");

  display.print("LSRFA:");
  display.print(lineSensorRFAnalogValue);
  display.print(", ");

  display.print("TSAV:");
  display.print(temperatureSensorAnalogValue);
  display.print(", ");

  display.print("BO:");
  display.print(buzzerOutput);
  display.print(", ");

  display.print("RGBRO:");
  display.print(rgbRedOutput);
  display.print(", ");

  display.print("RGBGO:");
  display.print(rgbGreenOutput);
  display.print(", ");

  display.print("RGBBO:");
  display.print(rgbBlueOutput);
  display.print(", ");

  display.display();

  // tone(BUZZER_PIN, buzzerOutput);
  analogWrite(RGB_LIGHT_MODULE_RED_PIN, rgbRedOutput);
  analogWrite(RGB_LIGHT_MODULE_GREEN_PIN, rgbGreenOutput);
  analogWrite(RGB_LIGHT_MODULE_BLUE_PIN, rgbBlueOutput);

  if (SERIAL_USE_CLEAR_MAGIC) {
    Serial.print(SERIAL_CLEAR_MAGIC_STRING);
  }

  Serial.println("");

  Serial.print("LSLFA:");
  Serial.print(lineSensorLFAnalogValue);
  Serial.print(", ");

  Serial.print("LSMFA:");
  Serial.print(lineSensorMFAnalogValue);
  Serial.print(", ");

  Serial.print("LSRFA:");
  Serial.print(lineSensorRFAnalogValue);
  Serial.print(", ");

  Serial.print("TSAV:");
  Serial.print(temperatureSensorAnalogValue);
  Serial.print(", ");

  Serial.print("BO:");
  Serial.print(buzzerOutput);
  Serial.print(", ");

  Serial.print("RGBRO:");
  Serial.print(rgbRedOutput);
  Serial.print(", ");

  Serial.print("RGBGO:");
  Serial.print(rgbGreenOutput);
  Serial.print(", ");

  Serial.print("RGBBO:");
  Serial.print(rgbBlueOutput);
  Serial.println(", ");
}

void setup() {
  Serial.begin(115200);

  // OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.setRotation(2);

  // Setup ultrasonic sensor
  pinMode(LEFT_SONIC_ECHO_PIN, INPUT);
  pinMode(LEFT_SONIC_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_SONIC_ECHO_PIN, INPUT);
  pinMode(RIGHT_SONIC_TRIG_PIN, OUTPUT);

  // Setup line sensor
  pinMode(LINE_SENSOR_LF_ANALOG_PIN, INPUT);
  pinMode(LINE_SENSOR_MF_ANALOG_PIN, INPUT);
  pinMode(LINE_SENSOR_RF_ANALOG_PIN, INPUT);

  pinMode(TEMPERATURE_SENSOR_PIN, INPUT);

  // pinMode(BUZZER_PIN, OUTPUT);

  pinMode(RGB_LIGHT_MODULE_BLUE_PIN, OUTPUT);
  pinMode(RGB_LIGHT_MODULE_GREEN_PIN, OUTPUT);
  pinMode(RGB_LIGHT_MODULE_RED_PIN, OUTPUT);

  // Wait for 5 seconds before starting
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Init... 5 secs");
  display.display();
  delay(5000);
}

void loop() { run_logic(); }
