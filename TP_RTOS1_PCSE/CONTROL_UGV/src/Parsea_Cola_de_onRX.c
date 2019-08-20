/*
 * Parsea_Cola_de_onRX.c
 *
 *  Created on: Aug 15, 2019
 *      Author: gzocco
 *
 *
 *      Tarea que recibe los bytes de la Cola xQueue_BTrx_frameParser
 *      los parsea y envia el comando recibido a la tarea de control de los motores
 *      por medio de la cola xQueue_frameParser_Control..
 */
#include "sapi.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "CONTROL_UGV.h"
#include "string.h"
#include "supporting_functions.h"
#include "Parsea_Cola_de_onRX.h"

DEBUG_PRINT_ENABLE;

extern QueueHandle_t xQueue_BTrx_frameParser;

void Parsea_Cola_de_onRX ( void* pvParameters )
{

	//Esta tarea recibe de una cola BYTE a BYTE recibidos por IRQ de UART.
	//onRxByte_t
	uint8_t lReceivedValue;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100UL );	// Son 100ms=100UL.

	uint8_t i=0;
	//char frame[MAX_MSG_SIZE]="";
	char rxMsgFrame[MAX_MSG_SIZE]="";
	msg_t lValueToSend;
	msg_t message;


	while(TRUE) {
		// Una tarea muy bloqueante para demostrar que la interrupcion funcina
		//gpioToggle(LEDB);
		//vTaskDelay(1000/portTICK_RATE_MS);
		if( uxQueueMessagesWaiting( xQueue_BTrx_frameParser ) != 0 ) {
			// vPrintString( "Queue should have been empty!\r\n" );
			xStatus = xQueueReceive( xQueue_BTrx_frameParser, &lReceivedValue, xTicksToWait);
			//xTicksToWait );

			if( xStatus == pdPASS ) {
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue.rxByte );
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue );
				//printf( "Recibimos <<%c>> por cola\r\n", lReceivedValue );
				//debugPrintChar(lReceivedValue);
				//debugPrintEnter();


				// Armo el string para parsearlo y armar la estructura de los comandos.

					// Tengo que hacer una pequeña capa de transporte con header, payload y tail para
					// eliminar los frame invalidos por motivos diversos....
					// Pasar lo mas posible a una tarea!

					if ( i < MAX_MSG_SIZE)	// Ver ahora? Arreglar para que no rompa cuando ingreso mas bytes del maximo!
					{
						// i va de 0 a 29, son 30 elementos en total!
						//printf ("i %d \n\r",i);
						rxMsgFrame[i]= lReceivedValue;	//uartRxRead( UART_232 );
						//uartWriteString(UART_PC,"A\r\n");
						if (rxMsgFrame[i] == '\n')	// Ver si al enviar CR LF son 2 bytes y genera problemas...
						{
							// Significa que llego un string completo.
							//strcpy(rxMsgFrame,frame);
							debugPrintString("Recibi frame: ");
							debugPrintString(rxMsgFrame);
							debugPrintEnter();

// Funcion de parsea de comandos por String.
							char* command = strtok(rxMsgFrame, SEPARADOR);
								//printf( "command: %s\r\n\r\n", command );

								//message.cmd[0][]="Test";
								//printf ("message.cmd %s\n\r",message.cmd[0]);

								// Para RTOS.

								//strcpy(message.cmd1,"Test");


								//	Chequeo si recibio mensaje completo de onRx.


								char msg[6][5]={{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0}};		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
								int j =0;

								while (command != NULL)		// Y si lo reemplazo por 1 for con i que recorra y cada 4 armo un message.cmdx. Y si i=5 no es coma lo descarto y fue?
								{
									printf ("Parseo %s\n\r",command);
									if (strlen(command) > 6)	// Ver bien el largo... Solo mira el primero, arreglar!!
									{
										// Ver por que rompe esto.
										printf ("Comando  %s demasiado largo. Invalido. \n\r",command);
										command = NULL;
										char rxMsgFrame[MAX_MSG_SIZE]="";
										break;
									}
									else
									{
										/*if (j>3)
										{
											strcpy (message.cmd1,command);
										}*/
										strcpy (msg[j],command);
										command= strtok (NULL, SEPARADOR);	// Aca falta verificar largo...
										j++;
									}
								}
								printf ("msg %s\n\r",msg[0]);
								printf ("msg %s\n\r",msg[1]);
								printf ("msg %s\n\r",msg[2]);
								printf ("msg %s\n\r",msg[3]);
								printf ("msg %s\n\r",msg[4]);
								printf ("msg %s\n\r",msg[5]);
								//printf ("message.cmd1 %s\n\r",message.cmd1);

								// Paso los datos del array a una estructura.
								strcpy(message.cmd0,msg[0]);
								strcpy(message.cmd1,msg[1]);
								strcpy(message.cmd2,msg[2]);
								strcpy(message.cmd3,msg[3]);
								strcpy(message.cmd4,msg[4]);
								strcpy(message.cmd5,msg[5]);

								vPrintTwoStrings( "Luego de copiar msg0 a message0  message0= ",message.cmd0 );
								vPrintTwoStrings( "Luego de copiar msg0 a message0  msg0= ",msg[0] );
								printf ("msg %s\n\r",msg[0]);
								printf ("messagecmd0 %s\n\r",message.cmd0);
								// Aca tiene que ir la cola que envia los datos a la tarea de control.

								lValueToSend = message;
								vPrintTwoStrings( "Send = ", lValueToSend.cmd0 );

								xStatus = xQueueSend( xQueue_frameParser_Control, &lValueToSend, xTicksToWait );

							// Reemplazar xQueueSendToBack por	xQueueSend. Segun la doc de FreeRtos es una macro.

								if( xStatus != pdPASS ) {
									/* We could not write to the queue because it was full � this must
								          be an error as the queue should never contain more than one item! */
									vPrintString( "Could not send to the queue.\r\n" );
								}

// Funcion de parsea de comandos por String.


							for (i = 0; i < MAX_MSG_SIZE; ++i)
							{
								rxMsgFrame[i] = 0;
							}
							i=0;

							// Aca debe avisar que termino de parsear un frame.
							//frame_rec=TRUE;
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
							rxMsgFrame[i] = 0;
						}
						i=0;
						//printf ("Descarto byte rx BT por mayor a MAX_MSG_SIZE i es %d \r\n ", i);
						//uartWriteByte( UART_USB, frame );
					}

			}
		}
	}
}
