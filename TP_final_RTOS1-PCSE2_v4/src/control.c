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
//#include "btControl.h"
// #include "btInit.h"
//#include "motorControl.h"

#include "hardwareInit.h"
#include "accionaMotores.h"
#include "hc05Bridge_Task.h"

/*==================[definiciones y macros]==================================*/

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



// NO se usa en el ejercicio de RTOS1 - PCSE.
// Para controlar maquina de estados que control el MODO DE OPERACION.
typedef enum{			// NO se usa en el ejercicio de RTOS1 - PCSE.
	INICIALIZANDO,
	READY,
	CONFIGURACION,
	RCONTROL,
	AUTONAV,
	CONLOST,
	ERR
} OpeModeFsmState_t;	// NO se usa en el ejercicio de RTOS1 - PCSE.
// Variable that hold the current state
OpeModeFsmState_t OpeModeFsmState;		// NO se usa en el ejercicio de RTOS1 - PCSE.
// Para controlar maquina de estados que control el MODO DE OPERACION.

typedef struct {		// NO se usa en el ejercicio de RTOS1 - PCSE.
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

// NO se usa en el ejercicio de RTOS1 - PCSE.
control_t controlData;	// Estructura de datos de control del UGV. // NO se usa en el ejercicio de RTOS1 - PCSE.

typedef struct {
	char cmd[6][5];		// Quitar luego de cambiar funcion parse.
	char cmd0[5];
	char cmd1[5];
	char cmd2[5];
	char cmd3[5];
	char cmd4[5];
	char cmd5[5];
} msg_t;


typedef struct {
	uint8_t rxByte;
} onRxByte_t;

// Para comunicacion por BT.
//char rxMsgFrame[MAX_MSG_SIZE]="";
bool_t frame_rec=FALSE;		// Quitar luego de implementar cola.
// Para comunicacion por BT.

/*==================[definiciones de datos internos de RTOS]=========================*/
QueueHandle_t xQueueBTrx_Control;		// Sacado de ejemplo 10 de FreeRTOS.
QueueHandle_t xQueue_BTrx_frameParse;		// Cola entre onRx y parseFrame. Modificando onRx para que envíe por colade a 1 byte.




TaskHandle_t xHandle_hc05Bridge_Task = NULL;	// Para manejar Delete de tarea Bridge AT.


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
//void btComTask ( void* taskParmPtr );
//void btInit(void);
void tarea_led__( void* taskParmPtr );
extern void hc05Bridge_Task ( void* taskParmPtr );

//void btComParse (void);

void OpeModeFsmInit( void );
void OpeModeFsmSet( OpeModeFsmState_t OpeMode);

extern void onRx( void *noUsado ); // ISR para recepcion BT.

//void btComParse (void);
//void comInterpreter(void);

OpeModeFsmState_t OpeModeFsmGet(void);
/*==================[declaraciones de funciones externas]====================*/
extern DesplazaFsmState_t DesplazaFsmState;
//extern void btInit (void);
extern void DesplazaFsmInit(void);
extern void DesplazaFsmUpdate(void);

extern void fsmButtonInit( void );
extern void fsmButtonUpdate( gpioMap_t tecla );
extern int hardwareInit (void);
extern void frameParser (void);
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
	boardConfig();
	hardwareInit ();

	// Falta definir que hacer si alguna de las colas no la puedo crear.
	xQueue_BTrx_frameParse = xQueueCreate( 120, sizeof( onRxByte_t ) );	// Cola para recepcion de UART dentro de IRQ.

	if( xQueue_BTrx_frameParse != NULL ) {
		debugPrintlnString( "xQueue_BTrx_frameParse Creada \n\r" );

	}else {
		debugPrintlnString( "Fallo Creacion xQueue_BTrx_frameParse \n\r" );
		/* The queue could not be created. */
	}

	xQueueBTrx_Control = xQueueCreate( 4, sizeof( msg_t ) );

	if( xQueueBTrx_Control != NULL ) {

		debugPrintlnString( "xQueueBTrx_Control Creada \n\r" );
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
				frameParser,                     // Funcion de la tarea a ejecutar
				(const char *)"frameParser",     // Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
				0,                          // Parametros de tarea
				tskIDLE_PRIORITY+2,         // Prioridad de la tarea
				0                           // Puntero a la tarea creada en el sistema
		);

/*		xTaskCreate(
				btComTask,                     // Funcion de la tarea a ejecutar
				(const char *)"btCom",     // Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
				0,                          // Parametros de tarea
				tskIDLE_PRIORITY+1,         // Prioridad de la tarea
				0                           // Puntero a la tarea creada en el sistema
		);
		*/


		//vTaskStartScheduler(); Lo prendo mas tarde.
	} else {

		debugPrintlnString( "Fallo Creacion xQueueBTrx_Control \n\r" );
		/* The queue could not be created. */
	}
/*
	xTaskCreate(
			hc05Bridge_Task,                     // Funcion de la tarea a ejecutar
			(const char *)"hc05Bridge_Task",     // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			0,                          // Parametros de tarea
			tskIDLE_PRIORITY+1,         // Prioridad de la tarea
			&xHandle_hc05Bridge_Task    // Puntero a la tarea creada en el sistema
	);
*/


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
	        	 if (strcmp(lReceivedValue.cmd0, "CFGB") == 0)
	        	 	   	{
	        		 xTaskCreate(
	        		 			hc05Bridge_Task,                     // Funcion de la tarea a ejecutar
	        		 			(const char *)"hc05Bridge_Task",     // Nombre de la tarea como String amigable para el usuario
	        		 			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	        		 			0,                          // Parametros de tarea
	        		 			tskIDLE_PRIORITY+5,         // Prioridad de la tarea
	        		 			&xHandle_hc05Bridge_Task    // Puntero a la tarea creada en el sistema
	        		 	);

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
	        	 	   					DesplazaFsmUpdate();
	        	 	   					uartWriteString( UART_BLUETOOTH, "ACK DERE\r\n" );
	        	 	   					}
	        	 	   					else
	        	 	   						if (strcmp(lReceivedValue.cmd0, "IZQU") == 0)
	        	 	   						{
	        	 	   							printf ("Comando LEFT \n\r");
	        	 	   							DesplazaFsmState = LEFT;
	        	 	   						DesplazaFsmUpdate();
	        	 	   						uartWriteString( UART_BLUETOOTH, "ACK IZQU\r\n" );
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

/*==================[fin del archivo]========================================*/
