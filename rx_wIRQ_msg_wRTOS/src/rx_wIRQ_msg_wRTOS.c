/*

You need to add

DEFINES+=TICK_OVER_RTOS
DEFINES+=USE_FREERTOS

on config.mk to tell SAPI to use FreeRTOS Systick


*/

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "sapi.h"
#include "string.h"
//#include "stdlib.h"

#define MAX_MSG_SIZE 30		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
							// Ver de usar una struct y calcular el tamaño con sizeof...
#define SEPARADOR ","

//char frame[MAX_MSG_SIZE]="";
char rxMsgFrame[MAX_MSG_SIZE]="";

bool msg_rec=FALSE;
//uint8_t i=0;		// ver de poner static dentro de onRX.

void tickTask( void* pvParameters )
{
	uint32_t param1=0;	// Para convertir a int
	uint32_t param2=0;
	char *eptr;	// Para convertir a int


	while(TRUE) {
		//gpioToggle(LEDB);
		if (msg_rec)
		{
			uartWriteString( UART_USB, rxMsgFrame);
			msg_rec=FALSE;
			char* command = strtok(rxMsgFrame, SEPARADOR);
			//printf( "command: %s\r\n\r\n", command );
			/*char msg[4][5]={{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0}};*/
			char msg[6][5]={{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0}};		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
			int j =0;

			while (command != NULL)
			{
				printf ("Parseo %s\n\r",command);
				if (strlen(command) > 6)	// Ver bien el largo... Solo mira el primero, arreglar.
				{
					// Ver por que rompe esto.
					printf ("Comado  %s demasiado largo. Invalido. \n\r",command);
					command = NULL;
					char rxMsgFrame[MAX_MSG_SIZE]="";
					break;
				}
				else
				{
					strcpy (msg[j],command);
					command= strtok (NULL, SEPARADOR);
					//printf ("msg %s\n",msg[0]);
					//printf ("cmd2 %s\n",msg[1]);
					j++;
				}
			}
			printf ("msg %s\n\r",msg[0]);
			printf ("msg %s\n\r",msg[1]);
			printf ("msg %s\n\r",msg[2]);
			printf ("msg %s\n\r",msg[3]);
			printf ("msg %s\n\r",msg[4]);
			printf ("msg %s\n\r",msg[5]);

			if (strcmp(msg[0], "CONF") == 0)
			{
				// Cambia el estado general segun el comando.
				printf ("Comando CONF \n\r");
				printf ("cmd: %s \n\r ", msg[0]);
				printf ("parametro 1: %s \n\r ", msg[1]);
				printf ("parametro 2: %s \n\r ", msg[2]);
				//printf ("parametro 1: %s ", msg[1]);
				//atoi()

				param1= strtol (msg[1],&eptr,10 );
				param2= strtol (msg[2],&eptr,10 );
				printf ("Integer de param1 %d \n\r ", param1 );
				printf ("Integer de param2 %d \n\r ", param2 );
				printf("Suma param1 param2 %d \n\r ",(param1+param2) );

			}
			else
				if (strcmp(msg[0], "READ") == 0)
				{
					printf ("Comando READY \n\r");
				}

				else /* default: */
				{
					printf ("Comando Incorrecto \n\r");
				}
		}
	}
	//printf( "Recibimos <<%s>> por UART\r\n", frame[] );
	vTaskDelay(100/portTICK_RATE_MS);
}


void onRx( void *noUsado )
{
	static uint8_t i=0;		// ver de poner static dentro de onRX.
	static char frame[MAX_MSG_SIZE]="";

	// Tengo que hacer una pequeña capa de transporte con header, payload y tail para
	// eliminar los frame invalidos por motivos diversos....

	if ( i < MAX_MSG_SIZE)	// Ver ahora? Arreglar para que no rompa cuando ingreso mas bytes del maximo!
	{
		// i va de 0 a 29, son 30 elementos en total!
		//printf ("i %d \n\r",i);
		frame[i]= uartRxRead( UART_232 );
		if (frame[i] == '\n')	// Ver si al enviar CR LF son 2 bytes y genera problemas...
		{
			// Significa que llego un string completo.
			strcpy(rxMsgFrame,frame);
			for (i = 0; i < MAX_MSG_SIZE; ++i)
					{
					  frame[i] = 0;
					}
			i=0;
			msg_rec=TRUE;
		}
		else
		{
			i++;
		}
	}
	else
	{
		// El string es mas largo de lo debido. Lo descarto.
		//printf ("Descarto byte rx BT por mayor a MAX_MSG_SIZE i es %d \r\n ", i);
		for (i = 0; i < MAX_MSG_SIZE; ++i)
		{
		  frame[i] = 0;
		}
		i=0;
		//printf ("Descarto byte rx BT por mayor a MAX_MSG_SIZE i es %d \r\n ", i);
		//uartWriteByte( UART_USB, frame );
	}
}

int main(void)
{
   /* Inicializar la placa */
   boardConfig();
   gpioInit( CAN_RD, GPIO_INPUT );	// Pin STATE del modulo BT HC-05.
   gpioInit( CAN_TD, GPIO_OUTPUT );	// Transistor ON/OFF BT HC-05.
   gpioInit( T_FIL0, GPIO_OUTPUT );	// PIN 34 de BT HC-05.
   gpioWrite (CAN_TD,HIGH);

   /* Inicializar la UART_USB junto con las interrupciones de Tx y Rx */
   uartConfig(UART_232, 9600);
   uartConfig(UART_USB, 9600);
   // Seteo un callback al evento de recepcion y habilito su interrupcion
   uartCallbackSet(UART_232, UART_RECEIVE, onRx, NULL);
   // Habilito todas las interrupciones de UART_USB
   uartInterrupt(UART_232, true);
   
   xTaskCreate(
      tickTask,                     // Funcion de la tarea a ejecutar
      (const char *)"tickTask",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+1,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );

   vTaskStartScheduler();

   return 0;
}
