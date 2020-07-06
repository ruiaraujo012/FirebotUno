/*
  FirebotUno
  Author: Rui Araújo
  Version: 0.2.0
*/
#include <Arduino.h>

#include <Servo.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

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
unsigned int echoMeasurement[4]; // Right, Front, Left, Not used (hold unacuareted values)
volatile byte echoMeasurementIndex;

/**
 * PID
 */
const double kp = 1;
const double ki = 0.00001;
const double kd = 10;
// const byte setpoint = 25;
const byte setpoint = 40;
double comulativePIDError;
double lastPIDError;
unsigned long previousPIDTime;

/**
 * Auxiliar variables
 */
unsigned long prevMillis;
unsigned int periodTime = 1000;

/**
 * LCD
 */
LiquidCrystal_I2C lcd(0x3F, 16, 2); // address, cols, rows

/**
 * Instantiate functions
 */
void move(int motorLeft, int motorRight);
void brake(byte motorR, byte motorL);
void sonarTrigger();
void echoSonarMeasurement();
void printOnLCD(String label1, String line1, String label2 = "", String line2 = "");
double computePID(int sensorInput);

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
  MsTimer2::set(50, sonarTrigger); // 50ms period
  MsTimer2::start();
  // MsTimer2::stop(); // Tests only

  // Echo interrupt
  attachInterrupt(digitalPinToInterrupt(echoSonar), echoSonarMeasurement, CHANGE);

  // Servo
  servoSonar.attach(9);
  servoSonar.write(10);

  // PID
  comulativePIDError = 0.0;
  lastPIDError = 0;
  previousPIDTime = 0;

  // Other variables
  state = true;
  trigger = HIGH;
  counter = 0;
  echoMeasurementIndex = 0;

  /**
   * LCD
   */
  lcd.init(); // initialize the lcd
  // lcd.backlight(); // turn on backlight
  lcd.noBacklight(); // turn off backlight

  prevMillis = millis();
}

// ******************************************
// *** Loop
// ******************************************
void loop()
{

  if (echoMeasurement[0] < 2000)
  {
    double PIDVal = computePID(echoMeasurement[0]);
    Serial.print(echoMeasurement[0]);
    Serial.print(" ");
    Serial.println(PIDVal);
    if (PIDVal < 0)
      move(100 - PIDVal, 200 + PIDVal);
    else
      move(100 + PIDVal, 200 - PIDVal);
  }

  // if (millis() - prevMillis >= periodTime)
  // {
  //   printOnLCD("Right: ", String(echoMeasurement[0]));

  //   prevMillis = millis();
  // }
}

/**
 *  Functions
 */

// Move robot wheels
void move(int motorLeft, int motorRight)
{
  motorLeft = constrain(motorLeft, -255, 255);
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
  /**
   * We will change later when we have state machine working.
   * If the state is (Follow Wall Right) only rotate servo to 0 -> 90 -> 0.
   * No need to search for left wall.
   * Right now, it stay 4 times on right and 2 times on front (repeatedly)
   */

  digitalWrite(triggerSonar, trigger);
  trigger = !trigger;
}

// External interrupt function to measure sonar distance
void echoSonarMeasurement()
{
  static unsigned long echo;
  unsigned int echoPinState = digitalRead(echoSonar);

  // 0, 1 -> Move
  // 2, 3, 4, 5, 6, 7, 8, 9 -> Right
  // 10, 11, -> Move
  // 12, 13 -> Front
  // if (counter > 13)
  //   counter = 0;

  // if (counter == 0)
  // {
  //   servoSonar.write(10);
  //   echoMeasurementIndex = 4;
  // }
  // else if (counter == 2)
  // {
  //   echoMeasurementIndex = 0;
  // }
  // else if (counter == 10)
  // {
  //   servoSonar.write(90);
  //   echoMeasurementIndex = 4;
  // }
  // else if (counter == 12)
  // {
  //   echoMeasurementIndex = 1;
  // }

  if (echoPinState)
  {
    echo = micros();
  }
  else
  {
    echoMeasurement[echoMeasurementIndex] = ((micros() - echo) >> 5); // Shift data to have a more small number
    // counter++;
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
double computePID(int sensorInput)
{
  unsigned long currentPIDTime = millis();

  int elapsedPIDTime = (int)(currentPIDTime - previousPIDTime);

  double PIDError = setpoint - sensorInput;

  comulativePIDError += PIDError * elapsedPIDTime;

  double ratePIDError = (PIDError - lastPIDError) / elapsedPIDTime;

  double outputPID = kp * PIDError + ki * comulativePIDError + kd * ratePIDError;
  outputPID = constrain(outputPID, -255, 255);

  lastPIDError = PIDError;
  previousPIDTime = currentPIDTime;

  return outputPID;
}