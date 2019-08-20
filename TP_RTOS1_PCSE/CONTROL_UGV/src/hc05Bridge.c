/*
 * hc05Bridge.c
 *
 *  Created on: Aug 15, 2019
 *      Author: gzocco
 *
 *      Funcion que pone el modulo HC-05 en modo comandos AT
 *      y la las UART en bridge para poder cursarle comandos AT
 *      desde la UART USB a la UART 232 y asi configurar o
 *      ver la configuracion del modulo.
 */
#include "hardwareInit.h"
#include "hc05Bridge.h"
#include "FreeRTOS.h"

#include "onRx.h"
#include "FreeRTOSConfig.h"

/* Demo includes. */
#include "supporting_functions.h"

void hc05Bridge ( void)
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
	/*if (hm10bleTest( UART_BLUETOOTH ) ){
		uartWriteString( UART_PC, "HC-05 Listo para recibir comandos AT:\r\n" );
	}
	else{
		uartWriteString( UART_PC, "HC-05 NOK.\r\n" );
	}*/
	//taskENTER_CRITICAL();
	//uartInterrupt(UART_232, false);		// Apago las IRQ de UART
	//vTaskSuspendAll ();
	while(TRUE)
	{
		if( uartReadByte( UART_PC, &data) ) {
			if( data == CARACTER_ESCAPE ) {		//"!" Es un caracter de escape para salir de la tarea.
				//taskEXIT_CRITICAL();
				//gpioWrite( LEDB, ON );
				uartWriteString(UART_USB,"Salgo modo Bridge AT por recibir CARACTER_ESCAPE.\r\n" );
				// Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
				gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
				gpioWrite(HC05_PIN34_AT_PIN,LOW);	// Pongo PIN 34 LOW
				vTaskDelay(800/portTICK_RATE_MS); // Para darle tiempo a que el HC-05 reaccione.
				gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
				uartConfig( UART_232, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
				uartWriteString(UART_USB, "BT en modo SPP a 9600.\r\n" );
				vTaskDelay(800/portTICK_RATE_MS);
				uartClearPendingInterrupt(UART_BLUETOOTH);
				uartCallbackClr( UART_BLUETOOTH, UART_RECEIVE );
				uartConfig(UART_232, 9600);
				// Seteo un callback al evento de recepcion y habilito su interrupcion
				uartCallbackSet(UART_232, UART_RECEIVE, onRx, NULL);
				// Habilito todas las interrupciones de UART_USB
				uartInterrupt(UART_232, true);

				//uartClearPendingInterrupt(UART_PC);
				// xQueueReset(xHandle_Recibe_Cola_de_onRX);
				//vTaskDelete(xHandle_Recibe_Cola_de_onRX); // Mato la tarea.
				//vTaskResume(xHandle_Parsea_Cola_de_onRX);
				// vTaskDelete(xHandle_hc05Bridge_Task);
				//uartInterrupt(UART_USB, true);		// Enciendo interrupciones
				//xTaskResumeAll ();
				//vTaskSuspend(xHandle_hc05Bridge_Task);
				break;
			}
			else
			{
				uartWriteByte( UART_232, data );
				//debugPrintlnString("Uart PC a BT: ");
				//debugPrintChar(data);
				//debugPrintEnter();
			}
		}

		if( uartReadByte( UART_232, &data ) ) {
			//debugPrintlnString("Uart BT a USB: ");
			//			debugPrintChar(data);
			//			debugPrintEnter();
		         uartWriteByte( UART_PC, data );
		      }
		//if( HC_modeAT != NULL ) {
/*
			uartWriteByte( UART_USB, HC_modeAT );
			debugPrintlnString("Uart BT a USB: ");
			debugPrintChar(HC_modeAT);
			debugPrintEnter();

			*/
	//	}
		//vTaskDelay(50/portTICK_RATE_MS);
	}
}
