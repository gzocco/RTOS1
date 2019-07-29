/*

You need to add

DEFINES+=TICK_OVER_RTOS
DEFINES+=USE_FREERTOS

on config.mk to tell SAPI to use FreeRTOS Systick

Hasta aca Logro recibir y separar un string de datos proveniendo de la UART
el string finaliza con ENTER (\n\r)
Tengo que corregir los chequeos por string mas prolongados que el array
y demas para que no rompa.
*/

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "sapi.h"
#include "string.h"

#define MAX_CMD_SIZE 20

char line[MAX_CMD_SIZE]="";
//=' ';
bool msg_rec=FALSE;
//char dato[20];
int i=0;

void tickTask( void* pvParameters )
{
	while(TRUE) {
		// Una tarea muy bloqueante para demostrar que la interrupcion funcina


		gpioToggle(LEDB);
		if (msg_rec)
		{
			uartWriteString( UART_USB, line);
			msg_rec=FALSE;
			char* command = strtok(line, "&");
			//printf( "command: %s\r\n\r\n", command );
			char cmd1[4][5]={{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0},
					{0,0,0,0,0}};
			int j =0;

			while (command != NULL)
			{
				printf ("Parseo %s\n\r",command);
				if (strlen(command) >4)
				{
					// Ver por que rompe esto.
					printf ("Comado  %s demasiado largo. Invalido. \n\r",command);
					command = NULL;
					char line[MAX_CMD_SIZE]="";
					break;
				}
				else
				{
					strcpy (cmd1[j],command);
					command= strtok (NULL, "&");
					//printf ("cmd1 %s\n",cmd1[0]);
					//printf ("cmd2 %s\n",cmd1[1]);
					j++;
				}
			}
				printf ("cmd1 %s\n\r",cmd1[0]);
				printf ("cmd2 %s\n\r",cmd1[1]);
				printf ("cmd3 %s\n\r",cmd1[2]);
				printf ("cmd4 %s\n\r",cmd1[3]);

				if (strcmp(cmd1[0], "CONF") == 0)
				{
					printf ("Comnado CONF \n\r");
				}
				else
					if (strcmp(cmd1[0], "READ") == 0)
					{
						printf ("Comnado READY \n\r");
					}
					else /* default: */
					{
						printf ("Comnado Incorrecto \n\r");
					}
			}					/*
      		   // Split the command in two values
      	       char* separator = strchr(command, ':');
      	       if (separator != 0)
      	       {
      	           // Actually split the string in 2: replace ':' with 0
      	    	 //uartWriteString( UART_USB, command);
			 *separator = 0;
      	          strcat(cmd1,command);
      	           //char scmd1ervoId = command;
      	           //char servoId = atol(command);
      	           ++separator;
      	           int position = atoi(separator);
      	           //printf( "servoID %s\r\n\r\n", servoId );
      	         printf( "scmd1 %s\r\n\r\n", cmd1 );
      	           printf( "position: %d\r\n\r\n", position );

      	           // Do something with servoId and position
      	       }
      	       // Find the next command in input string
      	       command = strtok(0, "&");
      	       //printf( "Comando ingresado dentro loop: %s\r\n\r\n", command );
      	   }
			 */
		}
		//printf( "Recibimos <<%s>> por UART\r\n", line[] );
		vTaskDelay(100/portTICK_RATE_MS);
	}


void onRx( void *noUsado )
{
	if ( strlen(line) < MAX_CMD_SIZE-2)		// Arreglar para que no rompa cuando ingreso mas bytes del maximo!
	{
		line[i]= uartRxRead( UART_232 );
				if (line[i] == '\n')
				{
					// Significa que llego un string completo.
					i=0;
					//line[]=0;
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
		i=0;
		line[MAX_CMD_SIZE]= 0;
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
