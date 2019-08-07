/*
 * hardwareInit.c
 *
 *  Created on: Aug 6, 2019
 *      Author: gzocco
 */
#include "sapi.h"
#include "hardwareInit.h"

DEBUG_PRINT_ENABLE;

// Funcion de inicializacion de Hardware para utilizar en parte CRITICA o antes de iniciar el Scheduler.
// Retorna 0 si OK.
int hardwareInit (void)
{
	uint8_t retVal=0;
	boardConfig();		// Inicializo placa EduCIAA.
	// Configuro Pines de GPIO para control de modulo BT HC-05.
	gpioInit( HC05_STATE_PIN, GPIO_INPUT );	// Pin STATE del modulo BT HC-05.
	gpioInit( HC05_POWERTR_PIN, GPIO_OUTPUT );	// Transistor ON/OFF BT HC-05.
	gpioInit( HC05_PIN34_AT_PIN, GPIO_OUTPUT );	// PIN 34 de BT HC-05.

	// Config mensajes de Debug por UART USB.
	debugPrintConfigUart( UART_USB, 9600 );
	debugPrintlnString( "DEBUG: Pruebo modulo HC-05 con comandos AT. \n\r" );
	debugPrintlnString( "DEBUG: Entro en modo AT a 38400 baud. \n\r" );

	// Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
	gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
	gpioWrite(HC05_PIN34_AT_PIN,HIGH);	// Pongo PIN 34 HIGH
	gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
	uartConfig( UART_BLUETOOTH, 38400);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.

	delayInaccurateMs(800);		// Delay que no usa SysTick solo para utilizarse antes de entrar en Scheduler.

	if (hm10bleTest( UART_BLUETOOTH ) ){
		debugPrintlnString( "DEBUG: HC-05 OK. \n\r" );
		//uartWriteString( UART_PC, "DEBUG: HC-05 OK.\r\n" );
		debugPrintlnString( "DEBUG: Salgo de modo AT. UART a 9600 baud. \n\r" );
		// Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
		gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
		gpioWrite(HC05_PIN34_AT_PIN,LOW);	// Pongo PIN 34 LOW
		delayInaccurateMs(800);		// Delay que no usa SysTick solo para utilizarse antes de entrar en Scheduler.
		//vTaskDelay(500/portTICK_RATE_MS); // Para darle tiempo a que el HC-05 reaccione.
		gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
		uartConfig( UART_BLUETOOTH, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
		delayInaccurateMs(800);		// Delay que no usa SysTick solo para utilizarse antes de entrar en Scheduler.
		//vPrintString( "BT en modo SPP a 9600.\r\n" );
		retVal=0;
	}
	else{
		debugPrintlnString( "DEBUG: HC-05 NOK. \n\r" );
		//uartWriteString( UART_PC, "DEBUG: HC-05 NOK.\r\n" );
		retVal=1;
	}
	return retVal;
}

bool_t hm10bleTest( int32_t uart )
{
   uartWriteString( uart, "AT\r\n" );
   //delayInaccurateMs(800);
   return waitForReceiveStringOrTimeoutBlocking( uart,
                                                 "OK\r\n", strlen("OK\r\n"),
                                                 1000 );
}
