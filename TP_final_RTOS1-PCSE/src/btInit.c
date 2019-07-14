/*
 * btInit.c
 *
 *  Created on: Jun 18, 2019
 *      Author: gzocco
 */
#include "sapi.h"
#include "btControl.h"
#include "btInit.h"

void btInit (void)
{
	boardInit();
	// Inicializar GPIOs
	gpioInit( 0, GPIO_ENABLE );

	// Tendria que definir los pines en otro lado tipo #define EnA GPIOX
	gpioInit( T_COL1, GPIO_OUTPUT );
	gpioInit( GPIO4, GPIO_OUTPUT );
	gpioInit( GPIO6, GPIO_OUTPUT );	// o el GPIO0
	//gpioInit( GPIO0, GPIO_OUTPUT );
	gpioInit( GPIO8, GPIO_OUTPUT );
	gpioInit( GPIO5, GPIO_OUTPUT );
	gpioInit( GPIO7, GPIO_OUTPUT );

/* Cambie algunos pines para poner otras cosas.
	// Configuracion de pines de salida para Leds de la EDU-CIAA-NXP
	gpioInit( GPIO2, GPIO_OUTPUT );
	gpioInit( GPIO4, GPIO_OUTPUT );
	gpioInit( GPIO6, GPIO_OUTPUT );	// o el GPIO0
	gpioInit( GPIO0, GPIO_OUTPUT );
	gpioInit( GPIO8, GPIO_OUTPUT );
	gpioInit( GPIO5, GPIO_OUTPUT );
	gpioInit( GPIO7, GPIO_OUTPUT );
*/
	//Inicializacion de var para ticks.
	//delayInit( &tiempo1, 20);
	// Inicializar UART_USB para conectar a la PC
	uartConfig( UART_PC, 115200 );
	uartWriteString( UART_PC, "UART_PC configurada.\r\n" );

	// Inicializar UART_232 para conectar al modulo bluetooth
	uartConfig( UART_BLUETOOTH, 9600 );
	uartWriteString( UART_PC, "UART_BLUETOOTH para modulo Bluetooth configurada.\r\n");

	//uartWriteString( UART_PC, "Testeo si el modulo esta conectado enviando: AT\r\n" );
	//	if( hm10bleTest( UART_BLUETOOTH ) ){
	//		uartWriteString( UART_PC, "Modulo conectado correctamente.\r\n" );
	//	}
	//	else{
	//		uartWriteString( UART_PC, "No funciona.\r\n" );
	//	}
}
