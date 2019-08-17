/*
 * accionaMotores.h
 *
 *  Created on: Aug 7, 2019
 *      Author: gzocco
 */

#ifndef PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V3_INC_ACCIONAMOTORES_H_
#define PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V3_INC_ACCIONAMOTORES_H_

#include "hardwareInit.h"

// Ver si va ?
int motorControl (uint8_t direction, uint8_t speedMotor1, uint8_t speedMotor2);
void DesplazaFsmError( void );
void DesplazaFsmUpdate( void );
void DesplazaFsmInit( void );

typedef enum{
	FORWARD,
	BACKWARD,
	RIGHT,
	LEFT,
	MOTORS_STOP
} DesplazaFsmState_t;

DesplazaFsmState_t DesplazaFsmState;

void forward(uint8_t speedMotor1, uint8_t speedMotor2);
void backward(uint8_t speedMotor1, uint8_t speedMotor2);
void right(uint8_t speedMotor1, uint8_t speedMotor2);
void left(uint8_t speedMotor1, uint8_t speedMotor2);
void motors_stop(uint8_t speedMotor1, uint8_t speedMotor2);

#endif /* PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V3_INC_ACCIONAMOTORES_H_ */
