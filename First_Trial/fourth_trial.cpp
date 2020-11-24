//Inverted Pendulum with DC Motor and Rotary Encoder

#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>
#include <stdlib.h>
#include <stdio.h>

//pwmPin and inPin1, inPin2 are pins for the control of the motor driver
#define pwmPin 6
#define inPin1 4
#define inPin2 5
//phaseAPin phaseBPin are the inputs from the Rotary Encoder
#define phaseAPin 21
#define phaseBPin 22

#define posAPin 0
#define posBPin 2

//Variables for the Encoder
int encoderCounter = 0;
int posEncoderCounter = 0;
int lastAState = 0;
int lastBState = 0;
int lastPosA = 0;
int lastPosB = 0;
//Speed of the Motor
double maxSpeed = 100.0f;
//Variables for PID Controller
const int arrayLength = 5;
int integralValues[arrayLength] = { };
int integralValPos[arrayLength] = { };
double Kp = 7;
double Ki = 0;
double Kd = 4;
int desiredState = 0;
int desiredPos = 0;
int pidError = 0;
double pidSpeed = 0.0f;
int cartPos = 0;


//Stop the Movement of the motor
void stopMovement()
{
	digitalWrite(inPin1, LOW);
	digitalWrite(inPin2, LOW);
	softPwmWrite(pwmPin, 0);
}

//Counter for first phase of rotary encoder
void phaseAencode()
{
	int phaseA = digitalRead(phaseAPin);
	//if there is a change in the first phase and phaseB is different from the phaseA then add 1
	if (phaseA != lastAState)
	{
		if (digitalRead(phaseBPin) != phaseA)
		{
			encoderCounter ++;
		}
		else
		{
			encoderCounter --;
		}
		//std::cout << "Counter : " << encoderCounter << '\n';
	}
	lastAState = phaseA;
}

//Counter for second phase of rotary encoder
void phaseBencode()
{
	int phaseB = digitalRead(phaseBPin);
	if (phaseB != lastBState)
	{
		if (digitalRead(phaseAPin) == phaseB)
		{
			encoderCounter ++;
		}
		else
		{
			encoderCounter --;
		}
		//std::cout << "Counter : " << encoderCounter << '\n';
	}
	lastBState = phaseB;
}

//Counter for first phase of rotary encoder for Position of cart
void posAencode()
{
	int phaseA = digitalRead(posAPin);
	//if there is a change in the first phase and phaseB is different from the phaseA then add 1
	if (phaseA != lastPosA)
	{
		if (digitalRead(posBPin) != phaseA)
		{
			posEncoderCounter ++;
		}
		else
		{
			posEncoderCounter --;
		}
		//std::cout << "Counter : " << encoderCounter << '\n';
	}
	lastPosA = phaseA;
}


//Counter for second phase of rotary encoder for Position of cart
void posBencode()
{
	int phaseB = digitalRead(posBPin);
	if (phaseB != lastPosB)
	{
		if (digitalRead(posAPin) == phaseB)
		{
			posEncoderCounter ++;
		}
		else
		{
			posEncoderCounter --;
		}
		//std::cout << "Counter : " << encoderCounter << '\n';
	}
	lastPosB = phaseB;
}

void wiringSetup()
{
	//Setup WiringPi
	wiringPiSetup ();
	pinMode(inPin1, OUTPUT);
	pinMode(inPin2, OUTPUT);
	wiringPiISR(phaseAPin, INT_EDGE_BOTH,&phaseAencode);
	wiringPiISR(phaseBPin, INT_EDGE_BOTH,&phaseBencode);
	wiringPiISR(posAPin, INT_EDGE_BOTH,&posAencode);
	wiringPiISR(posBPin, INT_EDGE_BOTH,&posBencode);
	//PWM_OUTPUT to control the speed in which the motor will rotate
	pinMode(pwmPin, PWM_OUTPUT);
	softPwmCreate(pwmPin, 0, 100);
}


//this is for the pullup
void firstMovement()
{
	if (encoderCounter > 0)
	{
		digitalWrite(inPin1, LOW);
		digitalWrite(inPin2, HIGH);
		softPwmWrite(pwmPin, maxSpeed / 2);
	}
	else if (encoderCounter < 0)
	{
		digitalWrite(inPin1, HIGH);
		digitalWrite(inPin2, LOW);
		softPwmWrite(pwmPin, maxSpeed / 2);
	}
}

//Rotate integral Array
void integralArrayRotation(int array[5])
{
	for (int i = 0; i < arrayLength - 1; i++)
	{
		array[i] = array[i+1];
	}
}

//sum Array
int arraySum (int array[arrayLength])
{
	int sumElements = 0;
	for (int i = 0; i < arrayLength; i++)
	{
		sumElements += array[i];
	}
	return sumElements;
}

void motorMovement(double pidSpeed)
{
	if (pidSpeed > 0)
	{
		digitalWrite(inPin1,HIGH);
		digitalWrite(inPin2,LOW);
	}
	else if ( pidSpeed < 0)
	{
		digitalWrite(inPin1,LOW);
		digitalWrite(inPin2,HIGH);
		pidSpeed = -pidSpeed;
	}
	else
	{
		digitalWrite(inPin1,LOW);
		digitalWrite(inPin2,LOW);
	}
	if ((pidSpeed > 100))
	{
		softPwmWrite(pwmPin,100);
	}
	else
	{
		softPwmWrite(pwmPin,pidSpeed);
	}
}

//PID Controller for angle of pendulum
void pidControlAngle(auto duration)
{
	pidError = desiredState - encoderCounter;
	if (pidError != integralValues[3])
	{
		integralArrayRotation(integralValues);
		integralValues[4] = pidError;
	}
	int sumElements = arraySum(integralValues);
	pidSpeed = Kp * pidError + Ki * sumElements / 5 + Kd * ( pidError - integralValues[3]);
	motorMovement(pidSpeed);
	std::cout << "rod encoder : " << encoderCounter << " PosCounter : " << posEncoderCounter << '\n';


}

//fix counter not getting abnormal values
void counterFix()
{
	if (encoderCounter > 800)
	{
		encoderCounter -= 1600;
	}
	if (encoderCounter < -800)
	{
		encoderCounter += 1600;
	}
}

int main(void)
{
	wiringSetup();
	stopMovement();
	while(1)
	{
		auto start = std::chrono::high_resolution_clock::now();
		phaseAencode();
		phaseBencode();
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		counterFix();
		pidControlAngle(duration);
		cartPos -= pidSpeed;
	}
	return 0;
}
