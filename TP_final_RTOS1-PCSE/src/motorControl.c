/*=============================================================================
 * Author: Gustavo Zocco <gzocco@gmail.com>
 * Date: 2019/05/26
 * Version: 0.1
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "sapi.h"
#include "motorControl.h"
//#include "sapi.h"		//Ya esta en main.c

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
// Declaro las funciones que accionan los motores de diferente forma.
void forward(uint8_t speedMotor1, uint8_t speedMotor2);
void backward(uint8_t speedMotor1, uint8_t speedMotor2);
void right(uint8_t speedMotor1, uint8_t speedMotor2);
void left(uint8_t speedMotor1, uint8_t speedMotor2);
void motors_stop(uint8_t speedMotor1, uint8_t speedMotor2);

/*=====[Main function, program entry point after power on or reset]==========*/
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
