/*
 * onRx.c
 *
 *  Created on: Aug 15, 2019
 *      Author: gzocco
 *
 *
 *      ISR de recepcion de datos a traves de UART.
 *      Envio los bytes recibidos a la cola de FreeRTOS: xQueue_BTrx_frameParser .
 *
 */
#include "sapi.h"
#include "FreeRTOS.h"
#include "../../../_programs/RTOS1/TP_RTOS1_PCSE/CONTROL_UGV/inc/CONTROL_UGV.h"


DEBUG_PRINT_ENABLE;


void onRx( void *noUsado )
{
	uint8_t data=0;
	uint8_t lValueToSend;
	BaseType_t xStatus;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	data = uartRxRead( UART_232 );
	debugPrintChar(data);
	debugPrintEnter();
	lValueToSend= data;
	xStatus = xQueueSendFromISR( xQueue_BTrx_frameParser, &lValueToSend, &xHigherPriorityTaskWoken );

	if( xStatus != pdPASS ) {
		/* We could not write to the queue because it was full � this must
   		          be an error as the queue should never contain more than one item! */
		vPrintString( "xQueue_BTrx_FrameParser: Could not send to the queue.\r\n" );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
