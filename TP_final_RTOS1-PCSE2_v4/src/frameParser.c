/*
 * frameParser.c
 *
 *  Created on: Aug 8, 2019
 *      Author: gzocco
 */
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
/* Demo includes. */
#include "supporting_functions.h"

// sAPI header
#include "sapi.h"
#include "hardwareInit.h"
#include "frameParser.h"

void frameParser (void)
{
	onRxByte_t lReceivedValue;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100UL );	// Son 100ms=100UL.

	static uint8_t i=0;		// ver de poner static dentro de onRX.
	static char frame[MAX_MSG_SIZE]="";
	char rxMsgFrame[MAX_MSG_SIZE]="";

	static bool_t waitBT=1;
	static bool_t waitComandos=1;
	//frame_rec= TRUE;
	// Aca comienza tarea de recepcion de datos BT.
	while(TRUE)
	{

		if( uxQueueMessagesWaiting( xQueue_BTrx_frameParse ) != 0 ) {
							vPrintString( "Queue should have been empty!\r\n" );
						}

						xStatus = xQueueReceive( xQueue_BTrx_frameParse, &lReceivedValue, xTicksToWait );

						if( xStatus == pdPASS ) {
							/* Data was successfully received from the queue, print out the received
										            value. */
							vPrintTwoStrings( "Received from onRx to frameParser = ", lReceivedValue.rxByte );

							//vPrintTwoStrings( "Received1= ", lReceivedValue.cmd1 );

							/* Tengo que hacer una pequeña capa de transporte con header, payload y tail para
							 * eliminar los frame invalidos por motivos diversos....
							 */

							if ( i < MAX_MSG_SIZE)	// Ver ahora? Arreglar para que no rompa cuando ingreso mas bytes del maximo!
							{
								// i va de 0 a 29, son 30 elementos en total!
								//printf ("i %d \n\r",i);
								frame[i]= lReceivedValue.rxByte;
								if (frame[i] == '\n')	// Ver si al enviar CR LF son 2 bytes y genera problemas...
								{
									// Significa que llego un string completo.
									strcpy(rxMsgFrame,frame);

									/*
									 *	A partir de aca lo saque de la otra funcion.
									 *
									 */
									//frame_rec=FALSE;		// Esto se va cuando termine de ordenar las colas.
									char* command = strtok(rxMsgFrame, SEPARADOR);
									//printf( "command: %s\r\n\r\n", command );

									//message.cmd[0][]="Test";
									//printf ("message.cmd %s\n\r",message.cmd[0]);

									// Para RTOS.
									msg_t lValueToSend;
									BaseType_t xStatus;

									msg_t message;
									//strcpy(message.cmd1,"Test");


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
											printf ("Comado  %s demasiado largo. Invalido. \n\r",command);
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
											command= strtok (NULL, SEPARADOR);
											j++;
										}
									}

									/*
									 * Aca hay que modificar para que solo lo muestre cuando estoy en DEBUG.
									 */
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

									xStatus = xQueueSend( xQueueBTrx_Control, &lValueToSend, 0 );
									// Reemplazar xQueueSendToBack por	xQueueSend

									if( xStatus != pdPASS ) {
										/* We could not write to the queue because it was full � this must
													          be an error as the queue should never contain more than one item! */
										vPrintString( "Could not send to the queue.\r\n" );
									}
									for (i = 0; i < MAX_MSG_SIZE; ++i)
									{
										frame[i] = 0;
									}
									i=0;
									frame_rec=TRUE;		// Cuando termine las colas no se va a usar mas.
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
						} else {
							/* We did not receive anything from the queue even after waiting for 100ms.
										        	            This must be an error as the sending tasks are free running and will be
										        	            continuously writing to the queue. */
							vPrintString( "Could not receive from the queue xQueue_BTrx_frameParse.\r\n" );
						}


						//btComParse();
					//}
				}
/*

		if (!gpioRead (HC05_STATE_PIN)){
			waitComandos=1;
			if (waitBT)
			{
				gpioWrite(LED1,HIGH);
				//message.cmd[0]="Test";
				//	printf ("message.cmd %s\n\r",message.cmd);

				uartWriteString( UART_PC, "Esperando Conexion al BT.\r\n" );
				waitBT=0;
			}
		}
		else
		{
			waitBT=1;
			if (waitComandos)
			{
				uartWriteString( UART_BLUETOOTH, "Esperando comandos:\r\n" );
				waitComandos=0;
				gpioWrite(LED3,LOW);
			}
		//	if (frame_rec)// Entra si recibí string de hasta 30 bytes terminado con \n.
		//	{////////////
		 *
		 *
		 * Aca iba todo....
				*/
	}


