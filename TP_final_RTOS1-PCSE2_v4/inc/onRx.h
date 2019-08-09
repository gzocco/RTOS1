/*
 * onRx.h
 *
 *  Created on: Aug 8, 2019
 *      Author: gzocco
 */

#ifndef PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V4_INC_ONRX_H_
#define PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V4_INC_ONRX_H_
QueueHandle_t xQueue_BTrx_frameParse;

void onRx( void *noUsado );
typedef struct {
	uint8_t rxByte;		// Quitar luego de cambiar funcion parse.
} onRxByte_t;

#endif /* PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V4_INC_ONRX_H_ */
