/*
 * * Author: Gustavo Zocco <gzocco@gmail.com>
 * Date: 2019/07/28
 * Version: 0.1
 */

/*==================[inlcusiones]============================================*/

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

// Includes de btControl
#include "btControl.h"
#include "btInit.h"
#include "motorControl.h"

/*==================[definiciones y macros]==================================*/
//Para interaccion modulo BT HC-05.
#define HC05_STATE_PIN CAN_RD
#define HC05_POWERTR_PIN CAN_TD
#define HC05_PIN34_AT_PIN T_FIL0

//Para recepciones de frame por UART BT.
#define MAX_MSG_SIZE 30		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
							// Ver de usar una struct y calcular el tamaño con sizeof...
#define SEPARADOR ","

/*==================[definiciones de datos internos]=========================*/


//	Para debounce de TEC1
uint32_t counter = 0;
uint32_t get_t_pulsacion();
void  reset_t_pulsacion();
// Para debounce de TEC1

// Para controlar maquina de estados que control el MODO DE OPERACION.
typedef enum{
	INICIALIZANDO,
	READY,
	CONFIGURACION,
	RCONTROL,
	AUTONAV,
	CONLOST,
	ERR
} OpeModeFsmState_t;
// Variable that hold the current state
OpeModeFsmState_t OpeModeFsmState;
// Para controlar maquina de estados que control el MODO DE OPERACION.

typedef struct {
   bool btConState;		// Estado de conexion de BT, segun pin STATE
   uint8_t maxTwaitBT;	// Maximo t que espera la conexion de BT antes de pasar a modo NOCON2BASE.
   uint8_t tReport2Base;	// Periodicidad de reporte de datos a Base.
   uint8_t maxDutyPWM;		// Maximo duty cicle de PWM.
   uint8_t minDutyPWM;		// Maximo duty cicle de PWM.
   float maxDistanceObs;	// Maxima distancia a Obstaculo que soporta el sensor.
   float minDistanceObs;	// Minima distancia a Obstaculo que soporta el sensor.
   float dist2Obs;			// Distancia al obstaculo.

   char *first_name;
   char *last_name;
   float balance;
} control_t;

control_t controlData;	// Estructura de datos de control del UGV.

typedef struct {
	char cmd[6][5];
	char cmd0[5];
	char cmd1[5];
	char cmd2[5];
	char cmd3[5];
	char cmd4[5];
	char cmd5[5];
} msg_t;


// Para comunicacion por BT.
char rxMsgFrame[MAX_MSG_SIZE]="";
bool frame_rec=FALSE;
// Para comunicacion por BT.

/*==================[definiciones de datos internos de Colas]=========================*/
QueueHandle_t xQueueBTrx_Control;		// Sacado de ejemplo 10 de FreeRTOS.


//msg_t message;

/*={{0,0,0,0,0},
				{0,0,0,0,0},
				{0,0,0,0,0},
				{0,0,0,0,0},
				{0,0,0,0,0},
				{0,0,0,0,0}};
				// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
*/



/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
// Tareas de FreeRTOS.
void tarea_tecla__( void* taskParmPtr );
void motorControlTask ( void* taskParmPtr );
void btComTask ( void* taskParmPtr );
void btInit(void);
void tarea_led__( void* taskParmPtr );



void btComParse (void);

void OpeModeFsmInit( void );
void OpeModeFsmSet( OpeModeFsmState_t OpeMode);

void onRx( void *noUsado ); // ISR para recepcion BT.

void verifHC05 (void);		// Verificar con comando AT el modulo HC-05.
void btComParse (void);
void comInterpreter(void);

OpeModeFsmState_t OpeModeFsmGet(void);
/*==================[declaraciones de funciones externas]====================*/
extern DesplazaFsmState_t DesplazaFsmState;
extern void btInit (void);
extern void DesplazaFsmInit(void);
extern void DesplazaFsmUpdate(void);

extern void fsmButtonInit( void );
extern void fsmButtonUpdate( gpioMap_t tecla );
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
	boardConfig();
	btInit();
	// Control del modulo BT. HC-05.
	gpioInit( HC05_STATE_PIN, GPIO_INPUT );	// Pin STATE del modulo BT HC-05.
	gpioInit( HC05_POWERTR_PIN, GPIO_OUTPUT );	// Transistor ON/OFF BT HC-05.
	gpioInit( HC05_PIN34_AT_PIN, GPIO_OUTPUT );	// PIN 34 de BT HC-05.

	gpioWrite (HC05_POWERTR_PIN,HIGH);		// Enciendo modulo BT.

	// verifHC05(); Hasta hacer que ande o armarlo como una tarea de FreeRTOS.

	xQueueBTrx_Control = xQueueCreate( 4, sizeof( msg_t ) );

	if( xQueueBTrx_Control != NULL ) {
		// Crear tarea en freeRTOS
		xTaskCreate(
				motorControlTask,                     // Funcion de la tarea a ejecutar
				(const char *)"mControlTask",     // Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
				0,                          // Parametros de tarea
				tskIDLE_PRIORITY+1,         // Prioridad de la tarea
				0                           // Puntero a la tarea creada en el sistema
		);

		// Crear tarea en freeRTOS
		xTaskCreate(
				btComTask,                     // Funcion de la tarea a ejecutar
				(const char *)"btCom",     // Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
				0,                          // Parametros de tarea
				tskIDLE_PRIORITY+1,         // Prioridad de la tarea
				0                           // Puntero a la tarea creada en el sistema
		);


		//vTaskStartScheduler(); Lo prendo mas tarde.
	} else {
		/* The queue could not be created. */
	}


	DesplazaFsmInit();
	OpeModeFsmInit();	// Falta programar que puebe todas las cosas...

	// UART for debug messages
   debugPrintConfigUart( UART_USB, 9600 );
   debugPrintlnString( "Control con RTOS \n\r" );

  /* msg_t message;
   	strcpy(message.cmd1,"Test");

   	debugPrintlnString( "message.cmd1: \n\r" );
   	debugPrintlnString( message.cmd1 );
   	debugPrintlnString( "\n\r" );

   	btComParse();
   	*/

   uartConfig(UART_232, 9600);	// UART BT.
   uartConfig(UART_USB, 9600);
      // Seteo un callback al evento de recepcion y habilito su interrupcion
   uartCallbackSet(UART_232, UART_RECEIVE, onRx, NULL);	// UART BT.
      // Habilito todas las interrupciones de UART_USB
   uartInterrupt(UART_232, true); // UART BT.

   // Led para dar se�al de vida
   gpioWrite( LED3, ON );
   debugPrintlnString( "Inicializado. OK." );	// Ya deberia de haber hecho todas las verificaciones.
   OpeModeFsmSet(READY);

/* Voy a crear 2 tareas. 1 para accionar los motores y otra para recibir los comandos
por BT e interpretarlos. Las saque y las cree cuando creo la cola.
*/

      // Crear tarea en freeRTOS
      xTaskCreate(
    		  tarea_tecla__,                     // Funcion de la tarea a ejecutar
			  (const char *)"tarea_tecla__",     // Nombre de la tarea como String amigable para el usuario
			  configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			  0,                          // Parametros de tarea
			  tskIDLE_PRIORITY+2,         // Prioridad de la tarea
			  0                           // Puntero a la tarea creada en el sistema
      );

      // Crear tarea en freeRTOS
            xTaskCreate(
            		tarea_led__,                     // Funcion de la tarea a ejecutar
      			  (const char *)"tarea_led__",     // Nombre de la tarea como String amigable para el usuario
      			  configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      			  0,                          // Parametros de tarea
      			  tskIDLE_PRIORITY+2,         // Prioridad de la tarea
      			  0                           // Puntero a la tarea creada en el sistema
            );

   // Iniciar scheduler
   vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/



/*==================[definiciones de funciones externas]=====================*/
void vApplicationTickHook( void )
{
	counter++;
}


void OpeModeFsmInit( void )
{

	OpeModeFsmState = INICIALIZANDO;   // Set initial state
}

void OpeModeFsmSet( OpeModeFsmState_t OpeMode)
{
	OpeModeFsmState = OpeMode;
}

OpeModeFsmState_t OpeModeFsmGet()
{
	return OpeModeFsmState;
}

void tarea_led__( void* taskParmPtr )
{
	uint32_t t_pulsacion_local;	// De la tarea led.
	while(TRUE)
	{
		//Esta parte es de la tarea led que dejo aca para no romper con lo demas...
		//Es la que enciende el LED1 durante el tiempo que pulse TEC1 del ejemplo.
		// Desde ACA
		t_pulsacion_local =get_t_pulsacion();

		if( t_pulsacion_local ==0)
		{
			vTaskDelay( 100 / portTICK_RATE_MS );
		}
		else
		{
			gpioWrite( LED2, ON );
			// Envia la tarea al estado bloqueado durante 1 s (delay)
			vTaskDelay( t_pulsacion_local  );
			gpioWrite( LED1, OFF );

			reset_t_pulsacion();
		}
		// Hasta ACA
	}
}

void btComTask ( void* taskParmPtr )
{
	//uint8_t data = 0;	// Byte de recepcion de BT.

	//uint32_t param1=0;	// Para convertir a int
	//uint32_t param2=0;
	//char *eptr;	// Para convertir a int.



	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE)
	{
		static bool_t waitBT=1;
		static bool_t waitComandos=1;
		// Aca comienza tarea de recepcion de datos BT.
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
			if (frame_rec)// Entra si recibí string de hasta 30 bytes terminado con \n.
			{
				btComParse();

			}
		}
	}
}

// Implementacion de funcion de la tarea
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
	   if( uxQueueMessagesWaiting( xQueueBTrx_Control ) != 0 ) {
	            vPrintString( "Queue should have been empty!\r\n" );
	         }
	   xStatus = xQueueReceive( xQueueBTrx_Control, &lReceivedValue, xTicksToWait );

	         if( xStatus == pdPASS ) {
	            /* Data was successfully received from the queue, print out the received
	            value. */
	        	 vPrintTwoStrings( "Received0 = ", lReceivedValue.cmd0 );
	        	 vPrintTwoStrings( "Received1= ", lReceivedValue.cmd1 );
	        	 if (strcmp(lReceivedValue.cmd0, "CONF") == 0)
	        	 	   	{
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
	        	 	   			DesplazaFsmState = FORWARD;
	        	 	   			DesplazaFsmUpdate();
	        	 	   			uartWriteString( UART_BLUETOOTH, "ACK AVAN\r\n" );
	        	 	   		}
	        	 	   		else
	        	 	   			if (strcmp(lReceivedValue.cmd0, "STOP") == 0)
	        	 	   			{
	        	 	   				printf ("Comando STOP \n\r");
	        	 	   				DesplazaFsmState = MOTORS_STOP;
	        	 	   				DesplazaFsmUpdate();
	        	 	   				uartWriteString( UART_BLUETOOTH, "ACK STOP\r\n" );
	        	 	   			}
	        	 	   			else
	        	 	   				if (strcmp(lReceivedValue.cmd0, "RETR") == 0)
	        	 	   				{
	        	 	   					printf ("Comando BACKWARD \n\r");
	        	 	   					DesplazaFsmState = BACKWARD;
	        	 	   					DesplazaFsmUpdate();
	        	 	   					uartWriteString( UART_BLUETOOTH, "ACK RETR\r\n" );
	        	 	   				}
	        	 	   				else
	        	 	   					if (strcmp(lReceivedValue.cmd0, "DERE") == 0)
	        	 	   					{
	        	 	   						printf ("Comando RIGHT \n\r");
	        	 	   						DesplazaFsmState = RIGHT;
	        	 	   					}
	        	 	   					else
	        	 	   						if (strcmp(lReceivedValue.cmd0, "IZQ") == 0)
	        	 	   						{
	        	 	   							printf ("Comando LEFT \n\r");
	        	 	   							DesplazaFsmState = LEFT;
	        	 	   						}
	        	 	   						else /* default: */
	        	 	   						{
	        	 	   							printf ("Comando Incorrecto \n\r");
	        	 	   							uartWriteString( UART_BLUETOOTH, "Comando incorrecto\r\n" );
	        	 	   						}
	         } else {
	            /* We did not receive anything from the queue even after waiting for 100ms.
	            This must be an error as the sending tasks are free running and will be
	            continuously writing to the queue. */
	            vPrintString( "Could not receive from the queue.\r\n" );
	         }

	  // DesplazaFsmUpdate();





     // vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}


void tarea_tecla__( void* taskParmPtr )
{
   fsmButtonInit();

   // Tarea periodica cada 1 ms
   portTickType xPeriodicity =  1 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE)
   {
	   fsmButtonUpdate(TEC1);

      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}


void onRx( void *noUsado )
{
	static uint8_t i=0;		// ver de poner static dentro de onRX.
	static char frame[MAX_MSG_SIZE]="";

	// Tengo que hacer una pequeña capa de transporte con header, payload y tail para
	// eliminar los frame invalidos por motivos diversos....
	// Pasar lo mas posible a una tarea!

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
			frame_rec=TRUE;
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

void comInterpreter(void)
{

}

void btComParse (void)
{
	//strcpy(rxMsgFrame,"AVAN,1234,5678,9012\r\n");

	/*uint32_t param1=0;	// Para convertir a int
	uint32_t param2=0;
	char *eptr;	// Para convertir a int.*/

	uartWriteString( UART_USB, rxMsgFrame);
	frame_rec=FALSE;
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

	xStatus = xQueueSendToBack( xQueueBTrx_Control, &lValueToSend, 0 );
// Reemplazar xQueueSendToBack por	xQueueSend

	if( xStatus != pdPASS ) {
		/* We could not write to the queue because it was full � this must
	          be an error as the queue should never contain more than one item! */
		vPrintString( "Could not send to the queue.\r\n" );
	}
}

/*==================[fin del archivo]========================================*/
