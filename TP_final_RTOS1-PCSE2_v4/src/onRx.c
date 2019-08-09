/*
 * onRx.c
 *
 *  Created on: Aug 8, 2019
 *      Author: gzocco
 */

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
/* Demo includes. */
#include "supporting_functions.h"
#include "onRx.h"
#include "sapi.h"

void onRx( void *noUsado )
{
	uint8_t data=0;
	onRxByte_t lValueToSend;
	//onRxByte_t rxByte;
	BaseType_t xStatus;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	data = uartRxRead( UART_232 );
	uartWriteByte ( UART_USB, data);
	vPrintStringAndNumber("Send onRx data to= ", data );
//	vPrintTwoStrings( "Send onRx data to= ", data );
	//lValueToSend.rxByte= uartRxRead( UART_232 );
	lValueToSend.rxByte= data;

	vPrintTwoStrings( "Send onRx to= ", lValueToSend.rxByte );
	xStatus = xQueueSendFromISR( xQueue_BTrx_frameParse, &lValueToSend, 0 );
	//xStatus = xQueueSendFromISR( xQueue_BTrx_frameParse, &lValueToSend, &xHigherPriorityTaskWoken );

	if( xStatus != pdPASS ) {
		/* We could not write to the queue because it was full � this must
		          be an error as the queue should never contain more than one item! */
		vPrintString( "xQueue_BTrx_parseFrame: Could not send to the queue.\r\n" );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
