/*
 * accionaMotores.c
 *
 *  Created on: Aug 7, 2019
 *      Author: gzocco
 */

//#include "motorControl.h"
#include "accionaMotores.h"
#include "hardwareInit.h"

//DesplazaFsmState_t DesplazaFsmState;

void forward(uint8_t speedMotor1, uint8_t speedMotor2)
{
	gpioWrite(MOTOR1_IN1_PIN, HIGH);
	gpioWrite(MOTOR1_IN2_PIN, LOW);

	gpioWrite(MOTOR2_IN3_PIN, HIGH);
	gpioWrite(MOTOR2_IN4_PIN, LOW);

	gpioWrite(MOTOR1_ENA_PIN, HIGH);
	gpioWrite(MOTOR2_ENB_PIN, HIGH);

}

void backward(uint8_t speedMotor1, uint8_t speedMotor2)
{
	gpioWrite(MOTOR1_IN1_PIN, LOW);
	gpioWrite(MOTOR1_IN2_PIN, HIGH);

	gpioWrite(MOTOR2_IN3_PIN, LOW);
	gpioWrite(MOTOR2_IN4_PIN, HIGH);

	gpioWrite(MOTOR1_ENA_PIN, HIGH);
	gpioWrite(MOTOR2_ENB_PIN, HIGH);
}

void left(uint8_t speedMotor1, uint8_t speedMotor2)
{
	gpioWrite(MOTOR1_IN1_PIN, LOW);
	gpioWrite(MOTOR1_IN2_PIN, LOW);

	gpioWrite(MOTOR2_IN3_PIN, HIGH);
	gpioWrite(MOTOR2_IN4_PIN, LOW);

	gpioWrite(MOTOR1_ENA_PIN, LOW);
	gpioWrite(MOTOR2_ENB_PIN, HIGH);
}

void right(uint8_t speedMotor1, uint8_t speedMotor2)
{
	gpioWrite(MOTOR1_IN1_PIN, HIGH);
	gpioWrite(MOTOR1_IN2_PIN, LOW);

	gpioWrite(MOTOR2_IN3_PIN, LOW);
	gpioWrite(MOTOR2_IN4_PIN, LOW);

	gpioWrite(MOTOR1_ENA_PIN, HIGH);
	gpioWrite(MOTOR2_ENB_PIN, LOW);
}

//function for stopping motors
void motors_stop(uint8_t speedMotor1, uint8_t speedMotor2)
{
	gpioWrite(MOTOR1_IN1_PIN, LOW);
	gpioWrite(MOTOR1_IN2_PIN, LOW);

	gpioWrite(MOTOR2_IN3_PIN, LOW);
	gpioWrite(MOTOR2_IN4_PIN, LOW);

	gpioWrite(MOTOR1_ENA_PIN, LOW);
	gpioWrite(MOTOR2_ENB_PIN, LOW);
}


void DesplazaFsmError( void )
{
	// Error handler, example, restart FSM:
	DesplazaFsmState = MOTORS_STOP;
}

// FSM Initialize Function
void DesplazaFsmInit( void )
{
	// Example:
	// boardInit();          // Initialize hardware
	DesplazaFsmState = MOTORS_STOP;   // Set initial state
}

// FSM Update Sate Function
void DesplazaFsmUpdate( void )
{
	switch(DesplazaFsmState)
	{
	case MOTORS_STOP:
		motorControl(0,10,10);
		break;
	case FORWARD:
		motorControl(1,10,10);
		break;
	case BACKWARD:
		motorControl(2,10,10);
		break;
	case RIGHT:
		motorControl(3,10,10);
		break;
	case LEFT:
		motorControl(4,10,10);
		break;
	default:
		DesplazaFsmError();
		break;
	}
}

int motorControl(uint8_t direction, uint8_t speedMotor1, uint8_t speedMotor2)
{
	switch(direction)
	{
	case 0:
		motors_stop(0,0);
		break;
	case 1:
		forward (speedMotor1,speedMotor2);
		break;
	case 2:
		backward (speedMotor1,speedMotor2);
		break;
	case 3:
		right (speedMotor1,speedMotor2);
		break;
	case 4:
		left (speedMotor1,speedMotor2);
		break;
	default:
		printf("Algo esta mal si llegaste aca...");
		motors_stop(0,0);
	}
	return 0;
}

