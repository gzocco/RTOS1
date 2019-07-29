/*
 * forward.c
 *
 *  Created on: Jun 18, 2019
 *      Author: gzocco
 */
#include "sapi.h"
#include "motorControl.h"
// function for driving straight
void forward(uint8_t speedMotor1, uint8_t speedMotor2)
{
	gpioWrite(motor1Ini1, HIGH);
	gpioWrite(motor1Ini2, LOW);

	gpioWrite(motor2Ini3, HIGH);
	gpioWrite(motor2Ini4, LOW);

	gpioWrite(motor1EnA, HIGH);
	gpioWrite(motor2EnB, HIGH);
	// Para PWM. Por ahora solo pone HIGH el pin EnX
}
