FirebotUno

## Important Notes

In this project we use a Servo Library that uses Timer1. These timer controls the PWM pins 9 and 10 of arduino Uno. While using this librarie, analogWrite() to those pins will not work normally.
However, we test a servo motor on pin 9, witch is a pwm pin controlled by timer 1 and it work perfectly, so we will use this pin for further tests until it fail (what we wish its not happen).

In the same way, we use the Timer2 for controlling the sonar period. Equaly, these timer controls the PWM pins 3 and 11 of arduino Uno. While using this librarie, analogWrite() to those pins will not work normally.

More info [here](https://www.pjrc.com/teensy/td_libs_TimerOne.html) and [here](https://forum.arduino.cc/index.php?topic=328094.0).
