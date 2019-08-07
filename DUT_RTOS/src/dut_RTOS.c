/*

dut_RTOS: Para probar separadamente en RTOS.

20190802

You need to add

DEFINES+=TICK_OVER_RTOS
DEFINES+=USE_FREERTOS

on config.mk to tell SAPI to use FreeRTOS Systick


 */

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "supporting_functions.h"

#include "sapi.h"
#include "string.h"

#include "hardwareInit.h"

#define CARACTER_ESCAPE '!'

TaskHandle_t xHandle = NULL;

DEBUG_PRINT_ENABLE;

void hc05BridgeTask( void* pvParameters )
{
	uint8_t data = 0;	// Variable para envio / recepcion de bytes entre UART PC Y UART BT.

	vPrintString( "Inicio tarea de Configuracion por Bridge HC-05 en modo AT.\r\n" );
	vTaskDelay(1000/portTICK_RATE_MS);
	//vPrintTwoStrings( "Send = ", lValueToSend.cmd0 );

	// Para entrar en modo AT en el HC-05. VERSION:2.0-20100601.
	gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
	gpioWrite(HC05_PIN34_AT_PIN,HIGH);	// Pongo PIN 34 HIGH
	vTaskDelay(800/portTICK_RATE_MS);
	gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
	uartConfig( UART_BLUETOOTH, 38400);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
	vTaskDelay(800/portTICK_RATE_MS);
	vPrintString( "BT en modo AT a 38400.\r\n" );
	vTaskDelay(500/portTICK_RATE_MS);

	if (hm10bleTest( UART_BLUETOOTH ) ){
		uartWriteString( UART_PC, "Modulo conectado correctamente.\r\n" );
	}
	else{
		uartWriteString( UART_PC, "No funciona.\r\n" );
	}
	taskENTER_CRITICAL();
	// Luego apagar IRQ de UART aqui.
	while(TRUE)
	{
		gpioToggle(LED1);

		if( uartReadByte( UART_PC, &data ) ) {
			if( data == CARACTER_ESCAPE ) {		//"!" Es un caracter de escape para salir de la tarea.
				taskEXIT_CRITICAL();
				//gpioWrite( LEDB, ON );
				vPrintString( "Salgo modo Bridge por recibir /CARACTER_ESCAPE/ AT.\r\n" );

				vPrintString( "Saliendo de modo AT.\r\n" );
						// Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
						gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
						gpioWrite(HC05_PIN34_AT_PIN,LOW);	// Pongo PIN 34 LOW
						vTaskDelay(800/portTICK_RATE_MS); // Para darle tiempo a que el HC-05 reaccione.
						gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
						uartConfig( UART_BLUETOOTH, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
						vPrintString( "BT en modo SPP a 9600.\r\n" );
						vTaskDelay(800/portTICK_RATE_MS);
						gpioToggle(LED1);
						//break;
						// Luego Encender IRQ de UART aqui.
						vTaskDelete(NULL); //Mato la tarea
			}
			else
			{
				uartWriteByte( UART_BLUETOOTH, data );
			}
		}
		if( uartReadByte( UART_BLUETOOTH, &data ) ) {

			uartWriteByte( UART_PC, data );
		}
		vTaskDelay(500/portTICK_RATE_MS);
	}
	//taskEXIT_CRITICAL();
	//vTaskDelete(xHandle); //Mato la tarea
}

void tickTask( void* pvParameters )
{
	while(TRUE)
	{
		gpioToggle(LED1);

		vPrintString( "tICK TASK T.\r\n" );

	/*	//vPrintTwoStrings( "Send = ", lValueToSend.cmd0 );

			// Para entrar en modo AT en el HC-05. VERSION:2.0-20100601.
			gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
			gpioWrite(HC05_PIN34_AT_PIN,HIGH);	// Pongo PIN 34 HIGH
			gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
			uartConfig( UART_BLUETOOTH, 38400);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
			vTaskDelay(500/portTICK_RATE_MS);
			vPrintString( "BT en modo AT a 38400.\r\n" );
			vTaskDelay(500/portTICK_RATE_MS);
			taskENTER_CRITICAL();

			if (hm10bleTest( UART_BLUETOOTH ) ){
				uartWriteString( UART_PC, "Modulo conectado correctamente.\r\n" );
			}
			else{
				uartWriteString( UART_PC, "No funciona.\r\n" );

			}

			taskEXIT_CRITICAL();

			vPrintString( "Saliendo de modo AT.\r\n" );
			// Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
			gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
			gpioWrite(HC05_PIN34_AT_PIN,LOW);	// Pongo PIN 34 LOW
			vTaskDelay(500/portTICK_RATE_MS); // Para darle tiempo a que el HC-05 reaccione.
			gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
			uartConfig( UART_BLUETOOTH, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
			vPrintString( "BT en modo SPP a 9600.\r\n" );
	}*/
	//printf( "Recibimos <<%s>> por UART\r\n", frame[] );
		vTaskDelay(1000/portTICK_RATE_MS);
}
	//vTaskDelay(600/portTICK_RATE_MS);
}

int main(void)
{


	debugPrintConfigUart( UART_USB, 9600 );
	debugPrintlnString( "DEBUG: Mensajes de DEBUG encendidos. \n\r" );

	hardwareInit();

	xTaskCreate(
			tickTask,                     // Funcion de la tarea a ejecutar
			(const char *)"tickTask",     // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			0,                          // Parametros de tarea
			tskIDLE_PRIORITY+2,         // Prioridad de la tarea
			0                           // Puntero a la tarea creada en el sistema
	);

	xTaskCreate(
				hc05BridgeTask,                     // Funcion de la tarea a ejecutar
				(const char *)"hc05BridgeTask",     // Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
				0,                          // Parametros de tarea
				tskIDLE_PRIORITY+1,         // Prioridad de la tarea
				&xHandle                           // Puntero a la tarea creada en el sistema
		);

	vTaskStartScheduler();

	 while( TRUE ) {
	      // Si cae en este while 1 significa que no pudo iniciar el scheduler
	   }

	return 0;
}
