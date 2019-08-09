/*
 * hc05Bridge_Task.c
 *
 *  Created on: Aug 7, 2019
 *      Author: gzocco
 */
#include "hc05Bridge_Task.h"
#include "hardwareInit.h"

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
/* Demo includes. */
#include "supporting_functions.h"

// sAPI header
#include "sapi.h"
#include "string.h"

extern TaskHandle_t xHandle_hc05Bridge_Task;

void hc05Bridge_Task ( void* taskParmPtr )
{
	uint8_t data = 0;	// Variable para envio / recepcion de bytes entre UART PC Y UART BT.
	vPrintString( "Creacion Tarea hc05Bridge_Task .\r\n" );
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
	vPrintString( "BT en modo AT a 38400. Se sale ingresando ! \r\n" );
	vTaskDelay(500/portTICK_RATE_MS);
	if (hm10bleTest( UART_BLUETOOTH ) ){
		uartWriteString( UART_PC, "HC-05 Listo para recibir comandos AT:\r\n" );
	}
	else{
		uartWriteString( UART_PC, "HC-05 NOK.\r\n" );
	}
	taskENTER_CRITICAL();
	uartInterrupt(UART_232, false);		// Apago las IRQ de UART
	while(TRUE)
	{
		if( uartReadByte( UART_PC, &data ) ) {
			if( data == CARACTER_ESCAPE ) {		//"!" Es un caracter de escape para salir de la tarea.
				taskEXIT_CRITICAL();
				//gpioWrite( LEDB, ON );
				vPrintString( "Salgo modo Bridge AT por recibir CARACTER_ESCAPE.\r\n" );
				// Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
				gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
				gpioWrite(HC05_PIN34_AT_PIN,LOW);	// Pongo PIN 34 LOW
				vTaskDelay(800/portTICK_RATE_MS); // Para darle tiempo a que el HC-05 reaccione.
				gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
				uartConfig( UART_BLUETOOTH, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
				vPrintString( "BT en modo SPP a 9600.\r\n" );
				vTaskDelay(800/portTICK_RATE_MS);
				gpioToggle(LED1);
				uartInterrupt(UART_232, true);		// Enciendo interrupciones
				vTaskDelete(xHandle_hc05Bridge_Task); // Mato la tarea.
			}
			else
			{
				uartWriteByte( UART_BLUETOOTH, data );
			}
		}
		if( uartReadByte( UART_BLUETOOTH, &data ) ) {

			uartWriteByte( UART_PC, data );
		}
		vTaskDelay(200/portTICK_RATE_MS);
	}
}

