#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>
#include <stdlib.h>
#include <stdio.h>


#define pwmPin 6
#define inPin1 4
#define inPin2 5

#define phaseAPin 21
#define phaseBPin 22
const int power = 100;

void wiringSetup(){
	wiringPiSetup();
	pinMode(inPin1, OUTPUT);
	pinMode(inPin2, OUTPUT);
	pinMode(pwmPin, PWM_OUTPUT);
	softPwmCreate(pwmPin, 0, 100);
}



int main(){
	wiringSetup();
	auto start = std::chrono::high_resolution_clock::now();
	int i = 1;
	while(i!=0){
	digitalWrite(inPin1,HIGH);
	digitalWrite(inPin2,LOW);
	softPwmWrite(pwmPin,power);
	std::cin >> i ;
	}
	digitalWrite(inPin1,LOW);
	digitalWrite(inPin2,LOW);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	std::cout << "Duration : " << duration.count() << " ms " << '\n';
}
