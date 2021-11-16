// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //

#define echoPin A6 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin A7 //attach pin D3 Arduino to pin Trig of HC-SR04

#define LEFT_SONIC_ECHO_PIN A10  
#define LEFT_SONIC_TRIG_PIN A11  

#define RIGHT_SONIC_ECHO_PIN A6  
#define RIGHT_SONIC_TRIG_PIN A7 

// defines variables
long left_duration; // variable for the duration of sound wave travel
float left_distance; // variable for the distance measurement

long right_duration; // variable for the duration of sound wave travel
float right_distance; // variable for the distance measurement

void setup() {
  pinMode(LEFT_SONIC_TRIG_PIN, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(LEFT_SONIC_ECHO_PIN, INPUT); // Sets the echoPin as an INPUT
  pinMode(RIGHT_SONIC_TRIG_PIN, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(RIGHT_SONIC_ECHO_PIN, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}
void loop() {
  // Clears the trigPin condition
  digitalWrite(LEFT_SONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(LEFT_SONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LEFT_SONIC_TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  left_duration = pulseIn(LEFT_SONIC_ECHO_PIN, HIGH);
  // Calculating the distance
  left_distance = left_duration * 0.034 / 2.0; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor

  // Clears the trigPin condition
  digitalWrite(RIGHT_SONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(RIGHT_SONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHT_SONIC_TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  right_duration = pulseIn(RIGHT_SONIC_ECHO_PIN, HIGH);
  // Calculating the distance
  right_distance = right_duration * 0.034 / 2.0; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  
  Serial.print("Left Distance: ");
  Serial.print(left_distance);
  Serial.print(", Right Distance: ");
  Serial.print(right_distance);
  Serial.println(" cm");
}
