/*
  FirebotUno
  Author: Rui Araújo
  Version: 0.1.0
*/
#include <Arduino.h>
#include <Servo.h>

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
 * Servo for Sonar
 */
Servo servoSonar;

unsigned long prevMillis;
unsigned int periodTime = 2000;
byte count = 0;

void servoSonarMove();
void move(int motorLeft, int motorRight);
void brake(byte motorR, byte motorL);

void setup()
{
  // Serial.begin(9600);
  pinMode(enableMotorRight, OUTPUT);
  pinMode(frontMotorRight, OUTPUT);
  pinMode(backMotorRight, OUTPUT);
  pinMode(enableMotorLeft, OUTPUT);
  pinMode(frontMotorLeft, OUTPUT);
  pinMode(backMotorLeft, OUTPUT);

  servoSonar.attach(6);
  prevMillis = millis();
}

void loop()
{

  if (millis() - prevMillis >= periodTime)
  {

    servoSonarMove();
    prevMillis = millis();
  }
}

// Functions
void servoSonarMove()
{
  switch (count)
  {
  case 0:
    servoSonar.write(10);
    count = 1;
    break;

  case 1:
    servoSonar.write(105);
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