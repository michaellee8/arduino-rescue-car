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

void log(char* str) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.print(str);
  display.display();

  Serial.println(str);
}

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
  const int DELAY_TIME = 10 * 1000;

  log("runnning all motors at 500 for 10 seconds");
  motor(500, 500, 500, 500);
  delay(DELAY_TIME);

  log("runnning all motors at -500 for 10 seconds");
  motor(-500, -500, -500, -500);
  delay(DELAY_TIME);

  log("running motorLF at 500 for 10 seconds");
  motor(500, 0, 0, 0);
  delay(DELAY_TIME);

  log("running motorLF at -500 for 10 seconds");
  motor(-500, 0, 0, 0);
  delay(DELAY_TIME);

  log("running motorLB at 500 for 10 seconds");
  motor(0, 500, 0, 0);
  delay(DELAY_TIME);

  log("running motorLB at -500 for 10 seconds");
  motor(0, -500, 0, 0);
  delay(DELAY_TIME);

  log("running motorRF at 500 for 10 seconds");
  motor(0, 0, 500, 0);
  delay(DELAY_TIME);

  log("running motorRF at -500 for 10 seconds");
  motor(0, 0, -500, 0);
  delay(DELAY_TIME);

  log("running motorRB at 500 for 10 seconds");
  motor(0, 0, 0, 500);
  delay(DELAY_TIME);

  log("running motorRB at -500 for 10 seconds");
  motor(0, 0, 0, -500);
  delay(DELAY_TIME);
}

void setup() {
  Serial.begin(115200);

  // OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.setRotation(2);

  // Wait for 5 seconds before starting
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Init... 5 secs");
  display.display();
  delay(5000);
}

void loop() { run_logic(); }
