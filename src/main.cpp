/*
  FirebotUno
  Author: Rui Araújo
  Version: 0.1.2
*/
#include <Arduino.h>

#include <Servo.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Temporary
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(0, 1); // RX | TX

/** 
 * Motors pin definition
 */
#define enableMotorRight 6 // ENB (PWM)
#define frontMotorRight 11 // IN4
#define backMotorRight 10  // IN3

#define enableMotorLeft 5 // ENA (PWM)
#define frontMotorLeft 8  // IN2
#define backMotorLeft 7   // IN1

#define minSpeed 120 // Minimum motor speed

/**
 * Servo and sonar
 */
#define triggerSonar 4
#define echoSonar 3 // Interrupt 1 (External)
#define servoLeftAngle 5

Servo servoSonar;

volatile bool state;
volatile byte trigger;
unsigned int echoMeasurement[4]; // Right, Front, Left, Not used (hold unacuareted values)
volatile byte echoMeasurementIndex;

/**
 * PID
 */
const float kp = 0.788;
const float ki = 0.0;
const float kd = 598.55;
const byte setpoint = 45;
float comulativePIDError;
float lastPIDError;
unsigned long previousPIDTime;

/**
 * LCD
 */
LiquidCrystal_I2C lcd(0x3F, 16, 2); // address, cols, rows

/**
 * Line Sensor
 */
#define sensorLineLeft A0
word sensorLineLeftValue;

/**
 * Auxiliar variables
 */
unsigned long prevMillis;
unsigned int periodTime = 1000;

/**
 * Instantiate functions
 */
void move(int motorLeft, int motorRight);
void brake(byte motorR, byte motorL);
void sonarTrigger();
void echoSonarMeasurement();
void printOnLCD(String label1, String line1, String label2 = "", String line2 = "");
float computePID(int sensorInput);

// ******************************************
// *** Setup
// ******************************************
void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);

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
  MsTimer2::set(25, sonarTrigger); // 25ms period
  MsTimer2::start();
  // MsTimer2::stop(); // Tests only

  // Echo interrupt
  attachInterrupt(digitalPinToInterrupt(echoSonar), echoSonarMeasurement, CHANGE);

  // Servo
  servoSonar.attach(9);
  servoSonar.write(servoLeftAngle);

  /**
   * PID
   */
  comulativePIDError = 0.0;
  lastPIDError = 0;
  previousPIDTime = 0;

  /**
   * LCD
   */
  lcd.init(); // initialize the lcd
  // lcd.backlight(); // turn on backlight
  lcd.noBacklight(); // turn off backlight

  /**
   * Line Sensor
   */
  pinMode(sensorLineLeft, INPUT);
  sensorLineLeftValue = 0;

  /**
   * Other variables
   */
  state = true;
  trigger = HIGH;
  echoMeasurementIndex = 0;

  prevMillis = millis();
}

// ******************************************
// *** Loop
// ******************************************
void loop()
{
  int sensorData = echoMeasurement[0];
  // if (sensorData < 1000)
  // {
  float PIDVal = computePID(sensorData);
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print(sensorData);
  Serial.print("\t");
  Serial.println(PIDVal);

  move(minSpeed - PIDVal, minSpeed + PIDVal);
  // }

  // if (PIDVal < 0)
  //   move(minSpeed - PIDVal, minSpeed + PIDVal);
  // else
  //   move(minSpeed - PIDVal, minSpeed + PIDVal);

  if (Serial.available())
    BTSerial.write(Serial.read());
}

/**
 *  Functions
 */

// Move robot wheels
void move(int motorLeft, int motorRight)
{
  motorLeft = motorLeft - 6; // Remove motor velocity error compared with right motor

  motorLeft = constrain(motorLeft, -249, 249);
  motorRight = constrain(motorRight, -255, 255);

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
  digitalWrite(triggerSonar, trigger);
  trigger = !trigger;
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
void printOnLCD(String label1, String line1, String label2 = "", String line2 = "")
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(label1);
  lcd.print(line1);

  if (label2 != "" && line2 != "")
  {

    lcd.setCursor(0, 1);
    lcd.print(label2);
    lcd.print(line2);
  }
}

// Compute PID
float computePID(int sensorInput)
{
  unsigned long currentPIDTime = millis();

  int elapsedPIDTime = (int)(currentPIDTime - previousPIDTime);

  float PIDError = setpoint - sensorInput;

  comulativePIDError += PIDError * elapsedPIDTime;

  float ratePIDError = (PIDError - lastPIDError) / elapsedPIDTime;

  float outputPID = kp * PIDError + ki * comulativePIDError + kd * ratePIDError;
  outputPID = constrain(outputPID, -255, 255);

  lastPIDError = PIDError;
  previousPIDTime = currentPIDTime;

  return outputPID;
}