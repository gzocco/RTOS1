/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

// sAPI header
#include "sapi.h"

// Includes de btControl
#include "btControl.h"
#include "btInit.h"
#include "motorControl.h"
extern DesplazaFsmState_t DesplazaFsmState;
extern void btInit (void);
extern void DesplazaFsmInit(void);
extern void DesplazaFsmUpdate(void);

extern void fsmButtonInit( void );
extern void fsmButtonUpdate( gpioMap_t tecla );
//

uint32_t counter = 0;

uint32_t get_t_pulsacion();
void  reset_t_pulsacion();

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void tarea_tecla__( void* taskParmPtr );
void motorControlTask ( void* taskParmPtr );
void btComTask ( void* taskParmPtr );
void btInit(void);
void tarea_led__( void* taskParmPtr );
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // Control del modulo BT. HC-05.
   gpioInit( CAN_RD, GPIO_INPUT );	// Pin STATE del modulo BT HC-05.
   gpioInit( CAN_TD, GPIO_OUTPUT );	// Transistor ON/OFF BT HC-05.
   gpioInit( T_FIL0, GPIO_OUTPUT );	// PIN 34 de BT HC-05.

   // Inicializo lo referido a btControl.
   btInit();
   DesplazaFsmInit();
   //

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );
   debugPrintlnString( "btControl con RTOS" );

   // Led para dar se�al de vida
   gpioWrite( LED3, ON );

/* Voy a crear 2 tareas. 1 para accionar los motores y otra para recibir los comandos
por BT e interpretarlos.
*/

   // Crear tarea en freeRTOS
   xTaskCreate(
      motorControlTask,                     // Funcion de la tarea a ejecutar
      (const char *)"mControlTask",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+1,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );

   // Crear tarea en freeRTOS
      xTaskCreate(
    	 btComTask,                     // Funcion de la tarea a ejecutar
         (const char *)"btCom",     // Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
         0,                          // Parametros de tarea
         tskIDLE_PRIORITY+1,         // Prioridad de la tarea
         0                           // Puntero a la tarea creada en el sistema
      );

      // Crear tarea en freeRTOS
      xTaskCreate(
    		  tarea_tecla__,                     // Funcion de la tarea a ejecutar
			  (const char *)"tarea_tecla__",     // Nombre de la tarea como String amigable para el usuario
			  configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			  0,                          // Parametros de tarea
			  tskIDLE_PRIORITY+2,         // Prioridad de la tarea
			  0                           // Puntero a la tarea creada en el sistema
      );

      // Crear tarea en freeRTOS
            xTaskCreate(
            		tarea_led__,                     // Funcion de la tarea a ejecutar
      			  (const char *)"tarea_led__",     // Nombre de la tarea como String amigable para el usuario
      			  configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      			  0,                          // Parametros de tarea
      			  tskIDLE_PRIORITY+2,         // Prioridad de la tarea
      			  0                           // Puntero a la tarea creada en el sistema
            );



   // Iniciar scheduler
   vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/



/*==================[definiciones de funciones externas]=====================*/
void vApplicationTickHook( void )
{
	counter++;
}

void tarea_led__( void* taskParmPtr )
{
	uint32_t t_pulsacion_local;	// De la tarea led.
	while(TRUE)
	{
		//Esta parte es de la tarea led que dejo aca para no romper con lo demas...
		//Es la que enciende el LED1 durante el tiempo que pulse TEC1 del ejemplo.
		// Desde ACA
		t_pulsacion_local =get_t_pulsacion();

		if( t_pulsacion_local ==0)
		{
			vTaskDelay( 100 / portTICK_RATE_MS );
		}
		else
		{
			gpioWrite( LED1, ON );
			// Envia la tarea al estado bloqueado durante 1 s (delay)
			vTaskDelay( t_pulsacion_local  );
			gpioWrite( LED1, OFF );

			reset_t_pulsacion();
		}
		// Hasta ACA
	}

}


void btComTask ( void* taskParmPtr )
{
	uint8_t data = 0;	// Byte de recepcion de BT.

	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE)
	{
		// Aca comienza tarea de recepcion de datos BT.
		if (!gpioRead (CAN_RD)){
			uartWriteString( UART_PC, "Nadie conectado al BT.\r\n" );
		}
		else
		{
			if( uartReadByte( UART_BLUETOOTH, &data ) ) {

				if( data == '0' ) {
					DesplazaFsmState = MOTORS_STOP;
					//motorControl (0, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				if( data == '1' ) {
					DesplazaFsmState = FORWARD;
					//motorControl (1, 10, 20);
					// gpioWrite( LEDB, OFF );
				}
				if( data == '2' ) {
					DesplazaFsmState = BACKWARD;
					//motorControl (2, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				if( data == '3' ) {
					DesplazaFsmState = RIGHT;
					//motorControl (3, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				if( data == '4' ) {
					DesplazaFsmState = LEFT;
					//motorControl (4, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				// Esto me va a servir para reenviar data de la UART BLUETOOTH a la PC para DEBUG.
				uartWriteString( UART_PC, "Byte Recibido por BT: \r\n" );
				uartWriteByte( UART_PC, data );

			}
		}
	}
}

// Implementacion de funcion de la tarea
void motorControlTask ( void* taskParmPtr )
{
   //fsmButtonInit();

   // Tarea periodica cada 1 ms
   portTickType xPeriodicity =  20 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE)
   {
	   DesplazaFsmUpdate();

      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}


void tarea_tecla__( void* taskParmPtr )
{
   fsmButtonInit();

   // Tarea periodica cada 1 ms
   portTickType xPeriodicity =  1 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE)
   {
	   fsmButtonUpdate(TEC1);

      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}
/*==================[fin del archivo]========================================*/
