/*
 * motorControlTask.c
 *
 *  Created on: Aug 15, 2019
 *      Author: gzocco
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "Parsea_Cola_de_onRX.h"
#include "sapi.h"
#include "CONTROL_UGV.h"
#include <string.h>
#include <sapi.h>
#include "hardwareInit.h"
#include "hc05Bridge.h"
#include "motorControlTask.h"

/* Demo includes. */
#include "supporting_functions.h"


extern QueueHandle_t xQueue_frameParser_Control;


void motorControlTask ( void* taskParmPtr )
{
	BaseType_t xStatus;
	msg_t lReceivedValue;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100UL );	// Son 100ms=100UL.
	uint32_t param1=0;	// Para convertir a int
	uint32_t param2=0;
	char *eptr;	// Para convertir a int.

	//fsmButtonInit();

   // Tarea periodica cada 1 ms
  // portTickType xPeriodicity =  20 / portTICK_RATE_MS;
  // portTickType xLastWakeTime = xTaskGetTickCount();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE)
   {
	   if( uxQueueMessagesWaiting( xQueue_frameParser_Control ) != 0 ) {
	            //vPrintString( "Queue should have been empty!\r\n" );

	   xStatus = xQueueReceive( xQueue_frameParser_Control, &lReceivedValue, xTicksToWait );

	         if( xStatus == pdPASS ) {
	            /* Data was successfully received from the queue, print out the received
	            value. */
	        	 vPrintTwoStrings( "Received0 = ", lReceivedValue.cmd0 );
	        	 vPrintTwoStrings( "Received1= ", lReceivedValue.cmd1 );
	        	 if (strcmp(lReceivedValue.cmd0, "CFGB") == 0)
	        	 	   	{
	        		 uartInterrupt(UART_232, false);
	        		 uartClearPendingInterrupt(UART_BLUETOOTH);
	        		 uartCallbackClr( UART_BLUETOOTH, UART_RECEIVE );


	        		 // vTaskSuspend(xHandle_Parsea_Cola_de_onRX);
	        		 hc05Bridge ();
	        	/*	 xTaskCreate(
	        		 	        		 	        		 		   Recibe_Cola_de_onRX,                     // Funcion de la tarea a ejecutar
	        		 	        		 	        		 		   (const char *)"Recibe_Cola_de_onRX",     // Nombre de la tarea como String amigable para el usuario
	        		 	        		 	        		 		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	        		 	        		 	        		 		   0,                          // Parametros de tarea
	        		 	        		 	        		 		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	        		 	        		 	        		 		   &xHandle_Recibe_Cola_de_onRX                           // Puntero a la tarea creada en el sistema
	        		 	        		 	        		    );


	        		 xTaskCreate(
	        		 	        		 			hc05Bridge_Task,                     // Funcion de la tarea a ejecutar
	        		 	        		 			(const char *)"hc05Bridge_Task",     // Nombre de la tarea como String amigable para el usuario
	        		 	        		 			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	        		 	        		 			0,                          // Parametros de tarea
	        		 	        		 			tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	        		 	        		 			&xHandle_hc05Bridge_Task    // Puntero a la tarea creada en el sistema
	        		 	        		 	);
*/


	        	//	 vTaskSuspendAll ();
	        		// taskENTER_CRITICAL();
	        		// configHC05 ();
	        		// xQueueReset( xQueueBTrx_Control );
	        		// uartInterrupt(UART_232, true);
	        		// 	 taskEXIT_CRITICAL();
	        		// 	xTaskResumeAll ();

	        		// vTaskResume(xHandle_hc05Bridge_Task);
	        		/*
	        		 *
	        		  xTaskCreate(
	        		 			hc05Bridge_Task,                     // Funcion de la tarea a ejecutar
	        		 			(const char *)"hc05Bridge_Task",     // Nombre de la tarea como String amigable para el usuario
	        		 			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	        		 			0,                          // Parametros de tarea
	        		 			tskIDLE_PRIORITY+2,         // Prioridad de la tarea
	        		 			&xHandle_hc05Bridge_Task    // Puntero a la tarea creada en el sistema
	        		 	);
	        		 */

	        		 // Cambia el estado general segun el comando.
	        	 	   		/*printf ("Comando CONF \n\r");
	        	 	   		printf ("cmd: %s \n\r ", lReceivedValue.cmd0);
	        	 	   		printf ("parametro 1: %s \n\r ", lReceivedValue.cmd1);
	        	 	   		printf ("parametro 2: %s \n\r ", lReceivedValue.cmd2);
	        	 	   		param1= strtol (lReceivedValue.cmd1,&eptr,10 );
	        	 	   		param2= strtol (lReceivedValue.cmd2,&eptr,10 );
	        	 	   		printf ("Integer de param1 %d \n\r ", param1 );
	        	 	   		printf ("Integer de param2 %d \n\r ", param2 );
	        	 	   		printf("Suma param1 param2 %d \n\r ",(param1+param2) );*/

	        	 	   	}
	        	 	   	else
	        	 	   		if (strcmp(lReceivedValue.cmd0, "AVAN") == 0)
	        	 	   		{
	        	 	   		vPrintString ("Comando AVAN \n\r");
	        	 	   			//DesplazaFsmState = FORWARD;
	        	 	   		//	DesplazaFsmUpdate();
	        	 	   		//	uartWriteString( UART_BLUETOOTH, "ACK AVAN\r\n" );
	        	 	   		}
	        	 	   		else
	        	 	   			if (strcmp(lReceivedValue.cmd0, "STOP") == 0)
	        	 	   			{
	        	 	   				printf ("Comando STOP \n\r");
	        	 	   			//	DesplazaFsmState = MOTORS_STOP;
	        	 	   			//	DesplazaFsmUpdate();
	        	 	   			//	uartWriteString( UART_BLUETOOTH, "ACK STOP\r\n" );
	        	 	   			}
	        	 	   			else
	        	 	   				if (strcmp(lReceivedValue.cmd0, "RETR") == 0)
	        	 	   				{
	        	 	   					printf ("Comando BACKWARD \n\r");
	        	 	   				//	DesplazaFsmState = BACKWARD;
	        	 	   				///	DesplazaFsmUpdate();
	        	 	   				//	uartWriteString( UART_BLUETOOTH, "ACK RETR\r\n" );
	        	 	   				}
	        	 	   				else
	        	 	   					if (strcmp(lReceivedValue.cmd0, "DERE") == 0)
	        	 	   					{
	        	 	   						printf ("Comando RIGHT \n\r");
	        	 	   					//	DesplazaFsmState = RIGHT;
	        	 	   					//DesplazaFsmUpdate();
	        	 	   					//uartWriteString( UART_BLUETOOTH, "ACK DERE\r\n" );
	        	 	   					}
	        	 	   					else
	        	 	   						if (strcmp(lReceivedValue.cmd0, "IZQU") == 0)
	        	 	   						{
	        	 	   							printf ("Comando LEFT \n\r");
	        	 	   					//		DesplazaFsmState = LEFT;
	        	 	   					//	DesplazaFsmUpdate();
	        	 	   					//	uartWriteString( UART_BLUETOOTH, "ACK IZQU\r\n" );
	        	 	   						}
	        	 	   						else /* default: */
	        	 	   						{
	        	 	   							printf ("Comando Incorrecto \n\r");
	        	 	   					//		uartWriteString( UART_BLUETOOTH, "Comando incorrecto\r\n" );
	        	 	   						}
	         } else {
	            /* We did not receive anything from the queue even after waiting for 100ms.
	            This must be an error as the sending tasks are free running and will be
	            continuously writing to the queue. */
	            vPrintString( "Could not receive from the queue.\r\n" );
	         }
	   }

	  // DesplazaFsmUpdate();





     // vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}