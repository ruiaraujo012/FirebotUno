/*
  FirebotUno
  Autor: Rui Araújo
  Versão: 0.1.0
*/

#include <Arduino.h>

/** 
 * Motors pin definition
*/
#define enableMotorRight 5 // ENA (PWM)
#define frontMotorRight 4  // IN3
#define backMotorRight 7   // IN4

#define enableMotorLeft 6 // ENB (PWM)
#define frontMotorLeft 8  // IN2
#define backMotorLeft 12  // IN1

void move(int motorRight, int motorLeft);
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
}

void loop()
{
  for (int i = 0; i < 255; i++)
  {
    move(i, i);
    delay(50);
  }

  for (int i = 255; i > 0; i--)
  {
    move(i, i);
    delay(50);
  }
}

// Functions
void move(int motorRight, int motorLeft)
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