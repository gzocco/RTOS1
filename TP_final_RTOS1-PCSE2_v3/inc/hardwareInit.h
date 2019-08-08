/*
 * hardwareInit.h
 *
 *  Created on: Aug 6, 2019
 *      Author: gzocco
 */

#ifndef PROGRAMS_RTOS1_DUT_RTOS_INC_HARDWAREINIT_H_
#define PROGRAMS_RTOS1_DUT_RTOS_INC_HARDWAREINIT_H_

#include "sapi.h"
#include "string.h"

#define UART_BLUETOOTH UART_232	// En esta UART esta el BT HC-05.
#define UART_PC UART_USB

// Definicion de Pines para control de giro de motores con modulo L298N.
#define MOTOR1_ENA_PIN	T_COL1  // P1.31	PWM5
#define MOTOR1_IN1_PIN	GPIO4	// P2.33
#define MOTOR1_IN2_PIN	GPIO6	// P2.35
#define MOTOR2_ENB_PIN	GPIO8	// P2.40    PWM6
#define MOTOR2_IN3_PIN	GPIO5	// P2.36
#define MOTOR2_IN4_PIN	GPIO7	// P2.38

// Definicion de pines para modulo BT HC-05.
#define HC05_STATE_PIN CAN_RD		// Pin STATE del modulo BT HC-05
#define HC05_POWERTR_PIN CAN_TD		// Pin de control de power con TR del HC-05
#define HC05_PIN34_AT_PIN T_FIL0	// Pin34 del HC-05

int hardwareInit (void);		// Funcion para inicializar el hardware.
bool_t hm10bleTest( int32_t uart );

#endif /* PROGRAMS_RTOS1_DUT_RTOS_INC_HARDWAREINIT_H_ */


