/*
 * esperaCom.c
 *
 *  Created on: Aug 17, 2019
 *      Author: gzocco
 *
 *      Tarea que verifica el estadp de conexion al modulo
 *      BT y reacciona segun el caso.
 */
#include "esperaCom.h"
#include "hardwareInit.h"
#include "FreeRTOS.h"
#include "accionaMotores.h"

void esperaCom ( void* taskParmPtr )
{
		// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE)
	{
		 static bool_t waitBT=1;
		 static bool_t waitComandos=1;

		if (!gpioRead (HC05_STATE_PIN))
		{
			waitComandos=1;
			gpioWrite(LED1,HIGH);
			if (waitBT)
			{
				DesplazaFsmState = MOTORS_STOP;		// Si se desconecta del BT, deja de accionar los motores.
				DesplazaFsmUpdate();
				uartWriteString( UART_PC, "Esperando Conexion al BT.\r\n" );
				waitBT=0;
			}
		}
		else
		{
			waitBT=1;
			gpioWrite(LED1,LOW);
			if (waitComandos)
			{
				uartWriteString( UART_BLUETOOTH, "Esperando comandos:\r\n" );
				waitComandos=0;
			}

		}
	}
	vTaskDelay(1000 / portTICK_RATE_MS);
}
