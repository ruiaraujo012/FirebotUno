/*
  FirebotUno
  Author: Rui Araújo
  Version: 0.2.0
*/
#include <Arduino.h>

#include <Servo.h>
#include <MsTimer2.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/** 
 * Motors pin definition
 */
#define enableMotorRight 6 // ENB (PWM)
#define frontMotorRight 11 // IN4
#define backMotorRight 10  // IN3

#define enableMotorLeft 5 // ENA (PWM)
#define frontMotorLeft 8  // IN2
#define backMotorLeft 7   // IN1

/**
 * Servo and sonar
 */
#define triggerSonar 4
#define echoSonar 3 // Interrupt 1 (External)

Servo servoSonar;

volatile bool state;
volatile byte trigger;
volatile byte counter;
unsigned long echoMeasurement[3]; // Right, Front, Left
volatile byte echoMeasurementIndex;

/**
 * Auxiliar variables
 */
unsigned long prevMillis;
unsigned int periodTime = 100;

/**
 * Display
 */
Adafruit_SSD1306 display(128, 64, &Wire, -1); // Screen width, screen height, &Wire, reset pin (This display does not have one)

/**
 * Instantiate functions
 */
void move(int motorLeft, int motorRight);
void brake(byte motorR, byte motorL);
void sonarTrigger();
void echoSonarMeasurement();
void printTwoLinesOnDisplay(char label1, int data1, char label2, int data2);
void printThreeLinesOnDisplay(char label1, int data1, char label2, int data2, char label3, int data3);

// ******************************************
// *** Setup
// ******************************************
void setup()
{
  Serial.begin(9600);

  /**
   * Motors
   */
  pinMode(enableMotorRight, OUTPUT);
  pinMode(frontMotorRight, OUTPUT);
  pinMode(backMotorRight, OUTPUT);
  pinMode(enableMotorLeft, OUTPUT);
  pinMode(frontMotorLeft, OUTPUT);
  pinMode(backMotorLeft, OUTPUT);

  /**
   * Servo and sonar
   */
  // Sonar pins
  pinMode(triggerSonar, OUTPUT);
  pinMode(echoSonar, INPUT);

  // Timer for sonar trigger
  MsTimer2::set(250, sonarTrigger); // 250ms period
  MsTimer2::start();
  // MsTimer2::stop(); // Tests only

  // Echo interrupt
  attachInterrupt(digitalPinToInterrupt(echoSonar), echoSonarMeasurement, CHANGE);

  // Servo
  servoSonar.attach(9);
  servoSonar.write(10);

  // Other variables
  state = false;
  trigger = HIGH;
  counter = 0;
  echoMeasurementIndex = 0;

  /**
   * Display
   */
  // initialize with the I2C addr 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // Clear the buffer.
  display.clearDisplay();

  prevMillis = millis();
}

// ******************************************
// *** Loop
// ******************************************
void loop()
{

  if (millis() - prevMillis >= periodTime)
  {
    // printTwoLinesOnDisplay('F', echoMeasurement[0], 'R', echoMeasurement[1]);
    printThreeLinesOnDisplay('R', echoMeasurement[0], 'F', echoMeasurement[1], 'L', echoMeasurement[2]);
    // Serial.print("Couter: ");
    // Serial.println(counter);

    // Serial.print("Echo R: ");
    // Serial.println(echoMeasurement[0]);

    // Serial.print("Echo F: ");
    // Serial.println(echoMeasurement[1]);

    // prevMillis = millis();
  }

  // move(150, 150);
}

/**
 *  Functions
 */

// Move robot wheels
void move(int motorLeft, int motorRight)
{
  motorRight = constrain(motorRight, -255, 255);
  motorLeft = constrain(motorLeft, -255, 255);

  if (motorRight < 0)
  {
    digitalWrite(frontMotorRight, LOW);
    digitalWrite(backMotorRight, HIGH);
    analogWrite(enableMotorRight, -motorRight);
  }
  else if (motorRight == 0)
  {
    digitalWrite(frontMotorRight, LOW);
    digitalWrite(backMotorRight, LOW);
    analogWrite(enableMotorRight, motorRight);
  }
  else
  {
    digitalWrite(frontMotorRight, HIGH);
    digitalWrite(backMotorRight, LOW);
    analogWrite(enableMotorRight, motorRight);
  }

  if (motorLeft < 0)
  {
    digitalWrite(frontMotorLeft, LOW);
    digitalWrite(backMotorLeft, HIGH);
    analogWrite(enableMotorLeft, -motorLeft);
  }
  else if (motorLeft == 0)
  {
    digitalWrite(frontMotorLeft, LOW);
    digitalWrite(backMotorLeft, LOW);
    analogWrite(enableMotorLeft, motorLeft);
  }
  else
  {
    digitalWrite(frontMotorLeft, HIGH);
    digitalWrite(backMotorLeft, LOW);
    analogWrite(enableMotorLeft, motorLeft);
  }
}

// Brake robot
void brake(byte motorR, byte motorL)
{

  if (motorR > 0)
  {
    digitalWrite(frontMotorRight, HIGH);
    digitalWrite(backMotorRight, HIGH);
    analogWrite(enableMotorRight, motorR);
  }

  if (motorL > 0)
  {
    digitalWrite(frontMotorLeft, HIGH);
    digitalWrite(backMotorLeft, HIGH);
    analogWrite(enableMotorLeft, motorL);
  }
}

// Trigger the sonar and move servo attached to him
// Timer interrupt function
void sonarTrigger()
{
  /**
   * We will change later when we have state machine working.
   * If the state is (Follow Wall Right) only rotate servo to 0 -> 90 -> 0.
   * No need to search for left wall.
   * Right now, it stay 4 times on right and 2 times on front (repeatedly)
   */
  switch (counter)
  {
  case 0:
    servoSonar.write(10);     // Rotate servo to the right
    echoMeasurementIndex = 0; // Index to save measured echo
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    break;

  case 1:
  case 3:
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    break;

  case 2:
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    break;

  case 4:
    servoSonar.write(90);     // Rotate servo to the front
    echoMeasurementIndex = 1; // Index to save measured echo
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    break;

  case 5:
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter = 0;
    break;

  default:
    break;
  }
}

// External interrupt function to measure sonar distance
void echoSonarMeasurement()
{
  static unsigned long echo;
  unsigned int echoPinState = digitalRead(echoSonar);

  if (echoPinState)
  {
    echo = micros();
  }
  else
  {
    echoMeasurement[echoMeasurementIndex] = ((micros() - echo) >> 5); // Shift data to have a more small number
  }
}

// Print two lines on display
void printTwoLinesOnDisplay(char label1, int data1, char label2, int data2)
{
  display.clearDisplay();

  display.drawRoundRect(0, 0, 128, 64, 2, WHITE);

  display.setTextSize(4);
  display.setTextColor(WHITE);

  display.setCursor(2, 2);
  display.print(label1);
  display.print(":");
  display.println(data1);

  display.drawLine(0, 31, 127, 31, WHITE);

  display.setCursor(2, 33);
  display.print(label2);
  display.print(":");
  display.println(data2);

  display.display();
}

// Print three lines on display
void printThreeLinesOnDisplay(char label1, int data1, char label2, int data2, char label3, int data3)
{
  display.clearDisplay();

  display.drawRoundRect(0, 0, 128, 64, 2, WHITE);

  display.setTextSize(2);
  display.setTextColor(WHITE);

  display.setCursor(2, 4);
  display.print(label1);
  display.print(":");
  display.println(data1);

  display.drawLine(0, 21, 127, 21, WHITE);

  display.setCursor(2, 25);
  display.print(label2);
  display.print(":");
  display.println(data2);

  display.drawLine(0, 42, 127, 42, WHITE);

  display.setCursor(2, 46);
  display.print(label3);
  display.print(":");
  display.println(data3);

  display.display();
}