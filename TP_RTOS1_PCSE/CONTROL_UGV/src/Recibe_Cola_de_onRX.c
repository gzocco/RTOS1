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
		// Una tarea muy bloqueante para demostrar que la interrupcion funcina
		gpioToggle(LEDB);
		//vTaskDelay(1000/portTICK_RATE_MS);
		if( uxQueueMessagesWaiting( xQueue_BTrx_frameParser ) != 0 ) {
			// vPrintString( "Queue should have been empty!\r\n" );
			xStatus = xQueueReceive( xQueue_BTrx_frameParser, &lReceivedValue, xTicksToWait);
			//xTicksToWait );

			if( xStatus == pdPASS ) {
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue.rxByte );
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue );
				//printf( "Recibimos <<%c>> por cola\r\n", lReceivedValue );

				debugPrintChar(lReceivedValue);
				debugPrintEnter();
				//HC_modeAT=lReceivedValue;

			}
		}
	}
}

