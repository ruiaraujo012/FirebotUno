/*
  FirebotUno
  Author: Rui Araújo
  Version: 0.2.0
*/
#include <Arduino.h>
#include <Servo.h>
#include <MsTimer2.h>

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
 * Instantiate functions
 */
void move(int motorLeft, int motorRight);
void brake(byte motorR, byte motorL);
void sonarTrigger();
void echoSonarMeasurement();

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

  prevMillis = millis();
}

// ******************************************
// *** Loop
// ******************************************
void loop()
{

  if (millis() - prevMillis >= periodTime)
  {
    Serial.print("Couter: ");
    Serial.println(counter);

    Serial.print("Echo R: ");
    Serial.println(echoMeasurement[0]);

    Serial.print("Echo F: ");
    Serial.println(echoMeasurement[1]);

    prevMillis = millis();
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
