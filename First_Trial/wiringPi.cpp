#include <wiringPi.h>
#include <iostream>
#include "constants.h"

void wiringSetup()
{
	wiringPiSetup();
	pinMode(inPin1, OUTPUT);
	pinMode(inPin2, OUTPUT);
	wiringPiISR(phaseAPin, INT_EDGE_BOTH, &phaseAencode);
	wiringPiISR(phaseBPin, INT_EDGE_BOTH, &phaseBencode);

	pinMode(pwmPin, PWM_OUTPUT);
	softPwmCreate(pwmPin, 0, 100);
}


