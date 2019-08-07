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

#define UART_BLUETOOTH UART_232
#define UART_PC UART_USB

#define HC05_STATE_PIN CAN_RD		// Pin STATE del modulo BT HC-05
#define HC05_POWERTR_PIN CAN_TD		// Pin de control de power con TR del HC-05
#define HC05_PIN34_AT_PIN T_FIL0	// Pin34 del HC-05

int hardwareInit (void);
bool_t hm10bleTest( int32_t uart );

#endif /* PROGRAMS_RTOS1_DUT_RTOS_INC_HARDWAREINIT_H_ */


