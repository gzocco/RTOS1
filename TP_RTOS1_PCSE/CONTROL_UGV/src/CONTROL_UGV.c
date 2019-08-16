/*=============================================================================
 * Author: Gustavo Zocco <gzocco@gmail.com>
 * Date: 2019/08/15
 * Version: 0.1
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/
#include "sapi.h"

//#include "../../../_programs/RTOS1/TP_RTOS1_PCSE/CONTROL_UGV/inc/userTasks.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "sapi.h"
#include "queue.h"
#include "string.h"
#include "hardwareInit.h"
#include "motorControlTask.h"
#include "Parsea_Cola_de_onRX.h"

#include "onRx.h"

#include "supporting_functions.h"
#include "../../../_programs/RTOS1/TP_RTOS1_PCSE/CONTROL_UGV/inc/CONTROL_UGV.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/






/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

DEBUG_PRINT_ENABLE;

int main( void )
{
	boardInit();
	hardwareInit ();
	debugPrintConfigUart( UART_USB, 9600 );
	debugPrintlnString( "Control con RTOS \n\r" );

	// Cola para recepcion de bytes por UART dentro de IRQ.
	xQueue_BTrx_frameParser = xQueueCreate( 100, sizeof( uint8_t ) );
	// Cola para enviar un mensaje parseado a la funcion de control.
	xQueue_frameParser_Control = xQueueCreate( 4, sizeof( msg_t ) );


	if( xQueue_BTrx_frameParser != NULL ) {
		debugPrintlnString( "xQueue_BTrx_frameParser Creada \n\r" );

	}else {
		debugPrintlnString( "Fallo Creacion xQueue_BTrx_frameParse \n\r" );
		/* The queue could not be created. */
	}
      /* Inicializar la UART_USB junto con las interrupciones de Tx y Rx */
      uartConfig(UART_232, 9600);
      // Seteo un callback al evento de recepcion y habilito su interrupcion
      uartCallbackSet(UART_232, UART_RECEIVE, onRx, NULL);
      // Habilito todas las interrupciones de UART_USB
      uartInterrupt(UART_232, true);

    /*
     *  Tarea que se puede utilizar en un futuro para armar una capa de transporte
     *  recibiendo bytes de la cola de onRx.
     *
     *
     xTaskCreate(
   		   Recibe_Cola_de_onRX,                     // Funcion de la tarea a ejecutar
   		   (const char *)"Recibe_Cola_de_onRX",     // Nombre de la tarea como String amigable para el usuario
   		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
   		   0,                          // Parametros de tarea
   		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
   		   0                           // Puntero a la tarea creada en el sistema
      );*/

      xTaskCreate(
    		  Parsea_Cola_de_onRX,                     // Funcion de la tarea a ejecutar
			  (const char *)"Recibe_Cola_de_onRX",     // Nombre de la tarea como String amigable para el usuario
			  configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			  0,                          // Parametros de tarea
			  tskIDLE_PRIORITY+1,         // Prioridad de la tarea
			  &xHandle_Parsea_Cola_de_onRX                           // Puntero a la tarea creada en el sistema
      );

      xTaskCreate(
    		  motorControlTask,                     // Funcion de la tarea a ejecutar
			  (const char *)"mControlTask",     // Nombre de la tarea como String amigable para el usuario
			  configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			  0,                          // Parametros de tarea
			  tskIDLE_PRIORITY+1,         // Prioridad de la tarea
			  0                           // Puntero a la tarea creada en el sistema
      );

      // Create a task in freeRTOS with dynamic memory
    /*
     *
     *
       xTaskCreate(
    		  myTask,                     // Function that implements the task.
			  (const char *)"myTask",     // Text name for the task.
			  configMINIMAL_STACK_SIZE*2, // Stack size in words, not bytes.
			  0,                          // Parameter passed into the task.
			  tskIDLE_PRIORITY+1,         // Priority at which the task is created.
			  0                           // Pointer to the task created in the system
      );*/

      vTaskStartScheduler();

      while( true ); // If reach heare it means that the scheduler could not start

      return 0;
   }
