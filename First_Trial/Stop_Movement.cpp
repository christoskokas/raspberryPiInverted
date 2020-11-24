#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

#define pwmPin 6
#define inPin1 4
#define inPin2 5

int main(void)
{
	wiringPiSetup();
	pinMode(inPin1, OUTPUT);
	pinMode(inPin2, OUTPUT);
	pinMode(pwmPin, PWM_OUTPUT);
	softPwmCreate(pwmPin, 0, 100);
	digitalWrite(inPin1, LOW);
	digitalWrite(inPin2, LOW);
	softPwmWrite(pwmPin, 0);
}
