/*
 * Recibe_Cola_de_onRX.c
 *
 *  Created on: Aug 15, 2019
 *      Author: gzocco
 *
 *      Esta tarea no se utiliza para la presente implementacion.
 */
#include "sapi.h"
#include "Recibe_Cola_de_onRX.h"
#include "FreeRTOS.h"

#include "../../../_programs/RTOS1/TP_RTOS1_PCSE/CONTROL_UGV/inc/CONTROL_UGV.h"

DEBUG_PRINT_ENABLE;

void Recibe_Cola_de_onRX( void* pvParameters )
{

	//Esta tarea recibe de una cola BYTE a BYTE recibidos por IRQ de UART.
	//onRxByte_t
	uint8_t lReceivedValue;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100UL );	// Son 100ms=100UL.

	while(TRUE) {
		gpioToggle(LEDB);
		if( uxQueueMessagesWaiting( xQueue_BTrx_frameParser ) != 0 ) {
			xStatus = xQueueReceive( xQueue_BTrx_frameParser, &lReceivedValue, xTicksToWait);
			if( xStatus == pdPASS ) {
				debugPrintChar(lReceivedValue);
				debugPrintEnter();
			}
		}
	}
}

