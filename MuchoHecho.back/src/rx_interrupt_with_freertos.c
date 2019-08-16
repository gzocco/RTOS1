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
#include "queue.h"
#include <string.h>
#include "hardwareInit.h"

/* Demo includes. */
#include "supporting_functions.h"


//Para recepciones de frame por UART BT.
#define MAX_MSG_SIZE 30		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
							// Ver de usar una struct y calcular el tamaño con sizeof...
#define SEPARADOR ","

#define CARACTER_ESCAPE '!'

QueueHandle_t xQueue_BTrx_frameParser;
QueueHandle_t xQueue_frameParser_Control;

TaskHandle_t xHandle_Parsea_Cola_de_onRX;
TaskHandle_t xHandle_Recibe_Cola_de_onRX;

TaskHandle_t xHandle_hc05Bridge_Task;

uint8_t HC_modeAT;


typedef struct {
	//char cmd[6][5];		// Quitar luego de cambiar funcion parse.
	char cmd0[5];
	char cmd1[5];
	char cmd2[5];
	char cmd3[5];
	char cmd4[5];
	char cmd5[5];
} msg_t;


DEBUG_PRINT_ENABLE;

void hc05Bridge_Task ( void* taskParmPtr );
extern int hardwareInit (void);

void Recibe_Cola_de_onRX( void* pvParameters )
{

	//Esta tarea recibe de una cola BYTE a BYTE recibidos por IRQ de UART.
	//onRxByte_t
	uint8_t lReceivedValue;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100UL );	// Son 100ms=100UL.

	while(TRUE) {
		// Una tarea muy bloqueante para demostrar que la interrupcion funcina
		gpioToggle(LEDB);
		//vTaskDelay(1000/portTICK_RATE_MS);
		if( uxQueueMessagesWaiting( xQueue_BTrx_frameParser ) != 0 ) {
			// vPrintString( "Queue should have been empty!\r\n" );
			xStatus = xQueueReceive( xQueue_BTrx_frameParser, &lReceivedValue, xTicksToWait);
			//xTicksToWait );

			if( xStatus == pdPASS ) {
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue.rxByte );
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue );
				//printf( "Recibimos <<%c>> por cola\r\n", lReceivedValue );

				debugPrintChar(lReceivedValue);
				debugPrintEnter();
				HC_modeAT=lReceivedValue;

			}
		}
	}
}


void Parsea_Cola_de_onRX( void* pvParameters )
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
		gpioToggle(LEDB);
		//vTaskDelay(1000/portTICK_RATE_MS);
		if( uxQueueMessagesWaiting( xQueue_BTrx_frameParser ) != 0 ) {
			// vPrintString( "Queue should have been empty!\r\n" );
			xStatus = xQueueReceive( xQueue_BTrx_frameParser, &lReceivedValue, xTicksToWait);
			//xTicksToWait );

			if( xStatus == pdPASS ) {
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue.rxByte );
				//vPrintTwoStrings( "Recibido por cola = ", lReceivedValue );
				//printf( "Recibimos <<%c>> por cola\r\n", lReceivedValue );
				debugPrintChar(lReceivedValue);
				debugPrintEnter();


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


void motorControlTask ( void* taskParmPtr )
{
	msg_t lReceivedValue;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100UL );	// Son 100ms=100UL.


	uint32_t param1=0;	// Para convertir a int
	uint32_t param2=0;
	char *eptr;	// Para convertir a int.

	//fsmButtonInit();

   // Tarea periodica cada 1 ms
  // portTickType xPeriodicity =  20 / portTICK_RATE_MS;
  // portTickType xLastWakeTime = xTaskGetTickCount();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE)
   {
	   if( uxQueueMessagesWaiting( xQueue_frameParser_Control ) != 0 ) {
	            //vPrintString( "Queue should have been empty!\r\n" );

	   xStatus = xQueueReceive( xQueue_frameParser_Control, &lReceivedValue, xTicksToWait );

	         if( xStatus == pdPASS ) {
	            /* Data was successfully received from the queue, print out the received
	            value. */
	        	 vPrintTwoStrings( "Received0 = ", lReceivedValue.cmd0 );
	        	 vPrintTwoStrings( "Received1= ", lReceivedValue.cmd1 );
	        	 if (strcmp(lReceivedValue.cmd0, "CFGB") == 0)
	        	 	   	{
	        		 vTaskSuspend(xHandle_Parsea_Cola_de_onRX);
	        		 xTaskCreate(
	        		 	        		 		   Recibe_Cola_de_onRX,                     // Funcion de la tarea a ejecutar
	        		 	        		 		   (const char *)"Recibe_Cola_de_onRX",     // Nombre de la tarea como String amigable para el usuario
	        		 	        		 		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	        		 	        		 		   0,                          // Parametros de tarea
	        		 	        		 		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	        		 	        		 		   &xHandle_Recibe_Cola_de_onRX                           // Puntero a la tarea creada en el sistema
	        		 	        		    );
	        		 xTaskCreate(
	        		 	        		 			hc05Bridge_Task,                     // Funcion de la tarea a ejecutar
	        		 	        		 			(const char *)"hc05Bridge_Task",     // Nombre de la tarea como String amigable para el usuario
	        		 	        		 			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	        		 	        		 			0,                          // Parametros de tarea
	        		 	        		 			tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	        		 	        		 			&xHandle_hc05Bridge_Task    // Puntero a la tarea creada en el sistema
	        		 	        		 	);
	        		 uartClearPendingInterrupt(UART_BLUETOOTH);
	        		 uartClearPendingInterrupt(UART_PC);
	        		 xQueueReset(xHandle_Recibe_Cola_de_onRX);
	        		 vTaskResume(xHandle_Parsea_Cola_de_onRX);


	        	//	 vTaskSuspendAll ();
	        		// taskENTER_CRITICAL();
	        		// configHC05 ();
	        		// xQueueReset( xQueueBTrx_Control );
	        		// uartInterrupt(UART_232, true);
	        		// 	 taskEXIT_CRITICAL();
	        		// 	xTaskResumeAll ();

	        		// vTaskResume(xHandle_hc05Bridge_Task);
	        		/*
	        		 *
	        		  xTaskCreate(
	        		 			hc05Bridge_Task,                     // Funcion de la tarea a ejecutar
	        		 			(const char *)"hc05Bridge_Task",     // Nombre de la tarea como String amigable para el usuario
	        		 			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	        		 			0,                          // Parametros de tarea
	        		 			tskIDLE_PRIORITY+2,         // Prioridad de la tarea
	        		 			&xHandle_hc05Bridge_Task    // Puntero a la tarea creada en el sistema
	        		 	);
	        		 */

	        		 // Cambia el estado general segun el comando.
	        	 	   		/*printf ("Comando CONF \n\r");
	        	 	   		printf ("cmd: %s \n\r ", lReceivedValue.cmd0);
	        	 	   		printf ("parametro 1: %s \n\r ", lReceivedValue.cmd1);
	        	 	   		printf ("parametro 2: %s \n\r ", lReceivedValue.cmd2);
	        	 	   		param1= strtol (lReceivedValue.cmd1,&eptr,10 );
	        	 	   		param2= strtol (lReceivedValue.cmd2,&eptr,10 );
	        	 	   		printf ("Integer de param1 %d \n\r ", param1 );
	        	 	   		printf ("Integer de param2 %d \n\r ", param2 );
	        	 	   		printf("Suma param1 param2 %d \n\r ",(param1+param2) );*/

	        	 	   	}
	        	 	   	else
	        	 	   		if (strcmp(lReceivedValue.cmd0, "AVAN") == 0)
	        	 	   		{
	        	 	   		vPrintString ("Comando AVAN \n\r");
	        	 	   			//DesplazaFsmState = FORWARD;
	        	 	   		//	DesplazaFsmUpdate();
	        	 	   		//	uartWriteString( UART_BLUETOOTH, "ACK AVAN\r\n" );
	        	 	   		}
	        	 	   		else
	        	 	   			if (strcmp(lReceivedValue.cmd0, "STOP") == 0)
	        	 	   			{
	        	 	   				printf ("Comando STOP \n\r");
	        	 	   			//	DesplazaFsmState = MOTORS_STOP;
	        	 	   			//	DesplazaFsmUpdate();
	        	 	   			//	uartWriteString( UART_BLUETOOTH, "ACK STOP\r\n" );
	        	 	   			}
	        	 	   			else
	        	 	   				if (strcmp(lReceivedValue.cmd0, "RETR") == 0)
	        	 	   				{
	        	 	   					printf ("Comando BACKWARD \n\r");
	        	 	   				//	DesplazaFsmState = BACKWARD;
	        	 	   				///	DesplazaFsmUpdate();
	        	 	   				//	uartWriteString( UART_BLUETOOTH, "ACK RETR\r\n" );
	        	 	   				}
	        	 	   				else
	        	 	   					if (strcmp(lReceivedValue.cmd0, "DERE") == 0)
	        	 	   					{
	        	 	   						printf ("Comando RIGHT \n\r");
	        	 	   					//	DesplazaFsmState = RIGHT;
	        	 	   					//DesplazaFsmUpdate();
	        	 	   					//uartWriteString( UART_BLUETOOTH, "ACK DERE\r\n" );
	        	 	   					}
	        	 	   					else
	        	 	   						if (strcmp(lReceivedValue.cmd0, "IZQU") == 0)
	        	 	   						{
	        	 	   							printf ("Comando LEFT \n\r");
	        	 	   					//		DesplazaFsmState = LEFT;
	        	 	   					//	DesplazaFsmUpdate();
	        	 	   					//	uartWriteString( UART_BLUETOOTH, "ACK IZQU\r\n" );
	        	 	   						}
	        	 	   						else /* default: */
	        	 	   						{
	        	 	   							printf ("Comando Incorrecto \n\r");
	        	 	   					//		uartWriteString( UART_BLUETOOTH, "Comando incorrecto\r\n" );
	        	 	   						}
	         } else {
	            /* We did not receive anything from the queue even after waiting for 100ms.
	            This must be an error as the sending tasks are free running and will be
	            continuously writing to the queue. */
	            vPrintString( "Could not receive from the queue.\r\n" );
	         }
	   }

	  // DesplazaFsmUpdate();





     // vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}
















void onRx( void *noUsado )
{
	uint8_t data=0;
	//onRxByte_t
	uint8_t lValueToSend;
	//onRxByte_t rxByte;
	BaseType_t xStatus;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	data = uartRxRead( UART_USB );
	//data = uartRxRead( UART_232 );
	//char c = uartRxRead( UART_USB);
  // printf( "Recibimos <<%c>> por UART\r\n", c );
  // printf( "Recibimos <<%c>> por UART\r\n", data );
   debugPrintChar(data);
   debugPrintEnter();

   //lValueToSend.rxByte= data;
   lValueToSend= data;
   xStatus = xQueueSendFromISR( xQueue_BTrx_frameParser, &lValueToSend, &xHigherPriorityTaskWoken );
   	//xStatus = xQueueSendFromISR( xQueue_BTrx_frameParse, &lValueToSend, &xHigherPriorityTaskWoken );

   	if( xStatus != pdPASS ) {
   		/* We could not write to the queue because it was full � this must
   		          be an error as the queue should never contain more than one item! */
   		vPrintString( "xQueue_BTrx_parseFramer: Could not send to the queue.\r\n" );
   	}

   portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


TaskHandle_t xHandle_hc05Bridge_Task;

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
		if( HC_modeAT != NULL ) {
			if( HC_modeAT == CARACTER_ESCAPE ) {		//"!" Es un caracter de escape para salir de la tarea.
				//taskEXIT_CRITICAL();
				//gpioWrite( LEDB, ON );
				vPrintString( "Salgo modo Bridge AT por recibir CARACTER_ESCAPE.\r\n" );
				// Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
				gpioWrite(HC05_POWERTR_PIN,LOW);	// Apago HC-05 BT
				gpioWrite(HC05_PIN34_AT_PIN,LOW);	// Pongo PIN 34 LOW
				vTaskDelay(800/portTICK_RATE_MS); // Para darle tiempo a que el HC-05 reaccione.
				gpioWrite(HC05_POWERTR_PIN,HIGH);	// Enciendo HC-05 BT
				uartConfig( UART_USB, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
				vPrintString( "BT en modo SPP a 9600.\r\n" );
				vTaskDelay(800/portTICK_RATE_MS);
				gpioToggle(LED1);
				//uartInterrupt(UART_USB, true);		// Enciendo interrupciones
				//xTaskResumeAll ();
				//vTaskSuspend(xHandle_hc05Bridge_Task);
				vTaskDelete(xHandle_Recibe_Cola_de_onRX); // Mato la tarea.
				vTaskDelete(xHandle_hc05Bridge_Task);
			}
			else
			{
				uartWriteByte( UART_USB, HC_modeAT );
			}
		}
		if( HC_modeAT != NULL ) {

			uartWriteByte( UART_USB, HC_modeAT );
		}
		vTaskDelay(200/portTICK_RATE_MS);
	}
}



int main(void)
{
   /* Inicializar la placa */
   boardConfig();
   hardwareInit ();
   debugPrintConfigUart( UART_USB, 9600 );
     debugPrintlnString( "Control con RTOS \n\r" );

	xQueue_BTrx_frameParser = xQueueCreate( 100, sizeof( uint8_t ) );	// Cola para recepcion de UART dentro de IRQ.
	xQueue_frameParser_Control = xQueueCreate( 4, sizeof( msg_t ) );


	if( xQueue_BTrx_frameParser != NULL ) {
		debugPrintlnString( "xQueue_BTrx_frameParser Creada \n\r" );

	}else {
		debugPrintlnString( "Fallo Creacion xQueue_BTrx_frameParse \n\r" );
		/* The queue could not be created. */
	}



   /* Inicializar la UART_USB junto con las interrupciones de Tx y Rx */
   uartConfig(UART_USB, 9600);
   // Seteo un callback al evento de recepcion y habilito su interrupcion
   uartCallbackSet(UART_USB, UART_RECEIVE, onRx, NULL);
   // Habilito todas las interrupciones de UART_USB
   uartInterrupt(UART_USB, true);
   
 /*  xTaskCreate(
		   Recibe_Cola_de_onRX,                     // Funcion de la tarea a ejecutar
		   (const char *)"Recibe_Cola_de_onRX",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
   );*/

   xTaskCreate(
		   Parsea_Cola_de_onRX,                     // Funcion de la tarea a ejecutar
   		   (const char *)"Recibe_Cola_de_onRX",     // Nombre de la tarea como String amigable para el usuario
   		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
   		   0,                          // Parametros de tarea
   		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   &xHandle_Parsea_Cola_de_onRX                           // Puntero a la tarea creada en el sistema
      );

   xTaskCreate(
   				motorControlTask,                     // Funcion de la tarea a ejecutar
   				(const char *)"mControlTask",     // Nombre de la tarea como String amigable para el usuario
   				configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
   				0,                          // Parametros de tarea
   				tskIDLE_PRIORITY+1,         // Prioridad de la tarea
   				0                           // Puntero a la tarea creada en el sistema
   		);

   vTaskStartScheduler();

   return 0;
}




int hardwareInit (void)
{
	uint8_t retVal=0;
	boardConfig();		// Inicializo placa EduCIAA.
	// Configuro Pines de GPIO para control de modulo BT HC-05.
	gpioInit( HC05_STATE_PIN, GPIO_INPUT );	// Pin STATE del modulo BT HC-05.
	gpioInit( HC05_POWERTR_PIN, GPIO_OUTPUT );	// Transistor ON/OFF BT HC-05.
	gpioInit( HC05_PIN34_AT_PIN, GPIO_OUTPUT );	// PIN 34 de BT HC-05.

	// Configuro pines para accionar los motores con el L298N.
	gpioInit( MOTOR1_ENA_PIN, GPIO_OUTPUT );
	gpioInit( MOTOR1_IN1_PIN, GPIO_OUTPUT );
	gpioInit( MOTOR1_IN2_PIN, GPIO_OUTPUT );
	gpioInit( MOTOR2_ENB_PIN, GPIO_OUTPUT );
	gpioInit( MOTOR2_IN3_PIN, GPIO_OUTPUT );
	gpioInit( MOTOR2_IN4_PIN, GPIO_OUTPUT );

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
