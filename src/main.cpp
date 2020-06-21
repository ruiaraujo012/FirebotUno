/*
  FirebotUno
  Author: Rui Araújo
  Version: 0.1.0
*/
#include <Arduino.h>
#include <Servo.h>
#include <TimerOne.h>

/** 
 * Motors pin definition
 */
#define enableMotorRight 5 // ENB (PWM)
#define frontMotorRight 8  // IN4
#define backMotorRight 7   // IN3

#define enableMotorLeft 3 // ENA (PWM)
#define frontMotorLeft 4  // IN2
#define backMotorLeft 2   // IN1

/**
 * Servo for Sonar and Sonar
 */
#define triggerSonar 9
#define ecoSonar 10
Servo servoSonar;
volatile bool state;
byte trigger;
byte counter;

/**
 * Auxiliar variables
 */
unsigned long prevMillis;
unsigned int periodTime = 10000;
byte count = 0;

/**
 * Instantiate functions
 */
void move(int motorLeft, int motorRight);
void brake(byte motorR, byte motorL);
void servoSonarMove();

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
   * Servo and Sonar
   */
  pinMode(triggerSonar, OUTPUT);
  pinMode(ecoSonar, INPUT);

  Timer1.initialize(5000000);
  Timer1.attachInterrupt(triggerSonar);
  interrupts();

  servoSonar.attach(6);
  servoSonar.write(0);

  state = false;
  trigger = HIGH;
  counter = 0;

  //prevMillis = millis();
}

void loop()
{

  // Serial.print("")

  // if (millis() - prevMillis >= periodTime)
  // {
  //   servoSonarMove();
  //   prevMillis = millis();
  // }

  // move(150, 150);
}

/**
 *  Functions
 */
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
  // else if (motorRight == 0)
  // {
  //   digitalWrite(frontMotorRight, LOW);
  //   digitalWrite(backMotorRight, LOW);
  //   analogWrite(enableMotorRight, motorRight);
  // }
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
  // else if (motorLeft == 0)
  // {
  //   digitalWrite(frontMotorLeft, LOW);
  //   digitalWrite(backMotorLeft, LOW);
  //   analogWrite(enableMotorLeft, motorLeft);
  // }
  else
  {
    digitalWrite(frontMotorLeft, HIGH);
    digitalWrite(backMotorLeft, LOW);
    analogWrite(enableMotorLeft, motorLeft);
  }
}

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

void servoSonarMove()
{
  Serial.println("Aqui");
  switch (count)
  {
  case 0:
    servoSonar.write(10);
    count = 1;
    break;

  case 1:
    servoSonar.write(90);
    count = 2;
    break;

  case 2:
    servoSonar.write(180);
    count = 3;
    break;

  case 3:
    servoSonar.write(90);
    count = 0;
    break;

  default:
    break;
  }
}

void sonarTrigger()
{
  /**
   * We will change later when we have state machine working.
   * If the state is (Follow Wall Right) only rotate servo to 0 -> 90 -> 0.
   * No need to search for left wall.
   */
  switch (counter)
  {
  case 0:
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    break;

  case 1:
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    servoSonar.write(90);
    break;

  case 2:
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    break;

  case 3:
    digitalWrite(triggerSonar, trigger);
    trigger = !trigger;
    counter++;
    servoSonar.write(0);
    counter = 0;
    state = true;
    break;
  }
}
