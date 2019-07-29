/*
 * backward.c
 *
 *  Created on: Jun 18, 2019
 *      Author: gzocco
 */
#include "sapi.h"
#include "motorControl.h"

//function for reversing
void backward(uint8_t speedMotor1, uint8_t speedMotor2)
{
	gpioWrite(motor1Ini1, LOW);
	gpioWrite(motor1Ini2, HIGH);

	gpioWrite(motor2Ini3, LOW);
	gpioWrite(motor2Ini4, HIGH);

	gpioWrite(motor1EnA, HIGH);
	gpioWrite(motor2EnB, HIGH);
}

