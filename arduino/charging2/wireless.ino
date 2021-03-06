#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define SAMPLE_SIZE 5

// ArduinoSort

#ifndef ArduinoSort_h
#define ArduinoSort_h

/**** These are the functions you can use ****/

// Sort an array
template<typename AnyType> void sortArray(AnyType array[], size_t sizeOfArray);

// Sort in reverse
template<typename AnyType> void sortArrayReverse(AnyType array[], size_t sizeOfArray);

// Sort an array with custom comparison function
template<typename AnyType> void sortArray(AnyType array[], size_t sizeOfArray, bool (*largerThan)(AnyType, AnyType));

// Sort in reverse with custom comparison function
template<typename AnyType> void sortArrayReverse(AnyType array[], size_t sizeOfArray, bool (*largerThan)(AnyType, AnyType));





/**** Implementation below. Do not use below functions ****/

namespace ArduinoSort {
  template<typename AnyType> bool builtinLargerThan(AnyType first, AnyType second) {
    return first > second;
  }

  template<> bool builtinLargerThan(char* first, char* second) {
    return strcmp(first, second) > 0;
  }

  template<typename AnyType> void insertionSort(AnyType array[], size_t sizeOfArray, bool reverse, bool (*largerThan)(AnyType, AnyType)) {
    for (size_t i = 1; i < sizeOfArray; i++) {
      for (size_t j = i; j > 0 && (largerThan(array[j-1], array[j]) != reverse); j--) {
        AnyType tmp = array[j-1];
        array[j-1] = array[j];
        array[j] = tmp;
      }
    }
  }
}

template<typename AnyType> void sortArray(AnyType array[], size_t sizeOfArray) {
  ArduinoSort::insertionSort(array, sizeOfArray, false, ArduinoSort::builtinLargerThan);
}

template<typename AnyType> void sortArrayReverse(AnyType array[], size_t sizeOfArray) {
  ArduinoSort::insertionSort(array, sizeOfArray, true, ArduinoSort::builtinLargerThan);
}

template<typename AnyType> void sortArray(AnyType array[], size_t sizeOfArray, bool (*largerThan)(AnyType, AnyType)) {
  ArduinoSort::insertionSort(array, sizeOfArray, false, largerThan);
}

template<typename AnyType> void sortArrayReverse(AnyType array[], size_t sizeOfArray, bool (*largerThan)(AnyType, AnyType)) {
  ArduinoSort::insertionSort(array, sizeOfArray, true, largerThan);
}


#endif

// ArduinoSort end

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV = 1, newV = 0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX

#define echoPinL A10
#define trigPinL A11
#define echoPinR A6
#define trigPinR A7

double distance_in_cmL;
double distance_in_cmR;

int pan = 90; // Left and right
int tilt = 108; // Up and down
long window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
int prev_volt = 0;
String if_sta = "";

#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20; //20
int servo_max = 160; //160

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 225;
int MIN_VALUE = 0;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define OFF_A +0     //Motor A Offset (Front Left)
#define OFF_B +1    //Motor B Offset (Front Right)
#define OFF_C +0     //Motor C Offset (Back Left)
#define OFF_D +1    //Motor D Offset (Back Right)

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm+OFF_A);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm+OFF_A);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm+OFF_B);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm+OFF_B);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm+OFF_C);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm+OFF_C);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm+OFF_D);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm+OFF_D);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   20000
#define MIN_PWM   300

int Motor_PWM = 60;


//    ???A-----B???
//     |  ???  |
//     |  |  |
//    ???C-----D???
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

//    ???A-----B???
//     |  |  |
//     |  ???  |
//    ???C-----D???
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B???
//     |   ??? |
//     | ???   |
//    ???C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

//    ???A-----B???
//     |  ???  |
//     |  ???  |
//    ???C-----D???
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    ???A-----B=
//     | ???   |
//     |   ??? |
//    =C-----D???
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    ???A-----B=
//     | ???   |
//     |   ??? |
//    =C-----D???
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    ???A-----B???
//     |  ???  |
//     |  ???  |
//    ???C-----D???
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B???
//     |   ??? |
//     | ???   |
//    ???C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

//    ???A-----B???
//     | ??? ??? |
//     | ??? ??? |
//    ???C-----D???
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

//    ???A-----B???
//     | ??? ??? |
//     | ??? ??? |
//    ???C-----D???
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

void UART_Control()
{
  String myString;
  char BT_Data = 0;
  // USB data
  /****
     Check if USB Serial data contain brackets
  */

  if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      // tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
 
    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    /*if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Serial_Data = ");
      display.println(myString);
      display.display();
    }*/

  }

  //BT Control
  /*
    Receive data from app and translate it to motor movements
  */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
}


/*Voltage Readings transmitter
  Sends them via Serial3*/
 void sendVolt() {
  newV = analogRead(A0);
  newV = newV * 25 / 1023;
  if (newV != oldV & newV != 0) {
    /*if (!Serial3.available()) {
      Serial3.println(newV);
      Serial.println(newV);
    }*/
    display.clearDisplay();
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("AI Robot");
    display.setCursor(0, 16);     // Start at top-left corner
    display.println("V:");
    display.setCursor(96, 16);     // Start at top-left corner
    display.println("V");
    display.setCursor(36, 16);     // Start at top-left corner
    display.println(newV);
    display.display();
  }
  oldV = newV;
}







//Where the program starts
void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
  servo_pan.attach(48);
  servo_tilt.attach(47);
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }/*
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("AI Robot");
  display.display();*/


  //Setup Voltage detector
  pinMode(A0, INPUT);

  //Setup ultrasonic sensor
  pinMode(echoPinL, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinR, OUTPUT);
}

void measure_distance() {
  long duration1;
  long duration2;
  long duration1s[SAMPLE_SIZE];
  long duration2s[SAMPLE_SIZE];
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  for (int i =0; i<SAMPLE_SIZE;i++){
    digitalWrite(trigPinL, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    
    duration1s[i] = pulseIn(echoPinL, HIGH);
    delayMicroseconds(5);
    
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);
  
    duration2s[i]= pulseIn(echoPinR, HIGH);
  }
  sortArray(duration1s, SAMPLE_SIZE);
  
  sortArray(duration2s, SAMPLE_SIZE);

  
  duration1 = duration1s[SAMPLE_SIZE/2];
  duration2 = duration2s[SAMPLE_SIZE/2];

  
  distance_in_cmL = (duration1 / 2.0) / 29.1;
  distance_in_cmR = (duration2 / 2.0) / 29.1;

    display.clearDisplay();
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("L");
    display.setCursor(72, 0);     // Start at top-left corner
    display.println("R");
    display.setCursor(24, 0);     // Start at top-left corner
    display.println(distance_in_cmL);
    display.setCursor(96, 0);     // Start at top-left corner
    display.println(distance_in_cmR);
    display.setCursor(0, 16);     // Start at top-left corner
    display.println(if_sta);
    display.display();

}

void loop() {
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
    UART_Control(); //get USB and BT serial data

    //constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);
  Motor_PWM = 50;

    //send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);

    //movement 001
    measure_distance();
    Serial.print(distance_in_cmL);
    Serial.print("    ");
    Serial.print(distance_in_cmR);
    Serial.println("");
         
  } if (voltCount >= 5) {
    // Next Task: Wireless Charging
    voltCount = 0;
    sendVolt();
    if (newV > 10) {STOP();} 
  } 

  // Find charger
  /*if (prev_volt == 1) {
    ADVANCE();
    delay(150);
    STOP();
    delay(50);
    prev_volt == 2;
  }*/
  if (distance_in_cmL < 7 & distance_in_cmR < 7 & abs(distance_in_cmL - distance_in_cmR) <= 2) {
    if (distance_in_cmL > 3.5 & distance_in_cmR > 3.5) {
      if_sta = "Fin Adv";
      ADVANCE();
      delay(30);
      STOP();
      delay(40);
    } else if (prev_volt == 0) {
      if_sta = "Fin Stop";
      prev_volt == 1;
    }
  } else if ((distance_in_cmL < 10 & distance_in_cmR > 10) | (distance_in_cmL < 7 & distance_in_cmR < 7 & distance_in_cmL < distance_in_cmR)) {
    // Left close, Right far, rotate left
    if_sta = "U TurnL";
    BACK(500, 500, 500, 500);
    delay(120);
    LEFT_2();
    delay(80);
    STOP();
    delay(100);
    rotate_1();
    delay(70);
    STOP();
    delay(100);
  } else if ((distance_in_cmL > 10 & distance_in_cmR < 10) | (distance_in_cmL < 7 & distance_in_cmR < 7 & distance_in_cmL > distance_in_cmR)) {
    // Left far, Right close, rotate right
    if_sta = "U TurnR";
    BACK(500, 500, 500, 500);
    delay(120);
    RIGHT_2();
    delay(70);
    STOP();
    delay(100);
    rotate_2();
    delay(50);
    STOP();
    delay(100);
  } else if (window_size < 15 & distance_in_cmL > 15 & distance_in_cmR > 15) {
    // No window 
    if_sta = "No Win";
    ADVANCE();
    delay(100);
    STOP();
    delay(200);
  } else if (distance_in_cmL > 15 | distance_in_cmR > 15) {
    // Both too far
    if_sta = "Win Adv";
    STOP();
    delay(100);
    ADVANCE();
    delay(100);
    STOP();
    delay(100);
    if (pan < 88) {
      // Cam panned to right, rotate right, shift left
      if_sta = "C TurnR";
      RIGHT_2();
      delay(50);
      rotate_2();
      delay(50);
      STOP();
      delay(100);
    } else if (pan > 93) {
      // Cam panned to left, rotate left, shift right
      if_sta = "C TurnL";
      LEFT_2();
      delay(50);
      rotate_1();
      delay(50);
      STOP();
      delay(100);
    }
  } else if ((distance_in_cmL > 7 & distance_in_cmL < 15) | (distance_in_cmR > 7 & distance_in_cmR < 15)) {
    // Camera and Ultrasonic both disabled 
    if_sta = "NULL Adv";
    ADVANCE();
    delay(100);
    STOP();
    delay(200);
  } else if (prev_volt == 0) {
    if_sta = "RLSH";
    prev_volt == 1;
  } else {
    if_sta = "AAAAAAAAAAAAAAAAA";
    STOP();
  }
     
  
}
