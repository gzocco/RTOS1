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

// sAPI header
#include "sapi.h"
#include "string.h"

// Includes de btControl
#include "btControl.h"
#include "btInit.h"
#include "motorControl.h"

extern DesplazaFsmState_t DesplazaFsmState;
extern void btInit (void);
extern void DesplazaFsmInit(void);
extern void DesplazaFsmUpdate(void);

extern void fsmButtonInit( void );
extern void fsmButtonUpdate( gpioMap_t tecla );
//
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

void OpeModeFsmInit( void );
void OpeModeFsmSet( OpeModeFsmState_t OpeMode);
OpeModeFsmState_t OpeModeFsmGet(void);

void onRx( void *noUsado ); // ISR para recepcion BT.
void verifHC05 (void);		// Verificar con comando AT el modulo HC-05.

// Para controlar maquina de estados que control el MODO DE OPERACION.


// Para comunicacion BT.
#define MAX_MSG_SIZE 30		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
							// Ver de usar una struct y calcular el tamaño con sizeof...
#define SEPARADOR ","
char rxMsgFrame[MAX_MSG_SIZE]="";
bool msg_rec=FALSE;
// Para comunicacion BT.

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void tarea_tecla__( void* taskParmPtr );
void motorControlTask ( void* taskParmPtr );
void btComTask ( void* taskParmPtr );
void btInit(void);
void tarea_led__( void* taskParmPtr );
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // Control del modulo BT. HC-05.
   gpioInit( CAN_RD, GPIO_INPUT );	// Pin STATE del modulo BT HC-05.
   gpioInit( CAN_TD, GPIO_OUTPUT );	// Transistor ON/OFF BT HC-05.
   gpioInit( T_FIL0, GPIO_OUTPUT );	// PIN 34 de BT HC-05.
   gpioWrite (CAN_TD,HIGH);		// Enciendo modulo BT.

   //verifHC05();

   // Inicializo lo referido a btControl.
   btInit();

   DesplazaFsmInit();
   OpeModeFsmInit();	// Falta programar que puebe todas las cosas...

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 9600 );
   debugPrintlnString( "Control con RTOS" );


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
por BT e interpretarlos.
*/

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
	// Example:
	// boardInit();          // Initialize hardware
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
			gpioWrite( LED1, ON );
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

	uint32_t param1=0;	// Para convertir a int
	uint32_t param2=0;
	char *eptr;	// Para convertir a int.


	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE)
	{
		static bool_t waitBT=1;
		static bool_t waitComandos=1;
		// Aca comienza tarea de recepcion de datos BT.
		if (!gpioRead (CAN_RD)){
			waitComandos=1;
			if (waitBT)
			{
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
			}
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
							param1= strtol (msg[1],&eptr,10 );
							param2= strtol (msg[2],&eptr,10 );
							printf ("Integer de param1 %d \n\r ", param1 );
							printf ("Integer de param2 %d \n\r ", param2 );
							printf("Suma param1 param2 %d \n\r ",(param1+param2) );

						}
						else
							if (strcmp(msg[0], "AVAN") == 0)
							{
								printf ("Comando AVAN \n\r");
								DesplazaFsmState = FORWARD;
								uartWriteString( UART_BLUETOOTH, "ACK AVAN\r\n" );
							}
							else
								if (strcmp(msg[0], "STOP") == 0)
								{
									printf ("Comando STOP \n\r");
									DesplazaFsmState = MOTORS_STOP;
								}
								else
									if (strcmp(msg[0], "RETR") == 0)
									{
										printf ("Comando BACKWARD \n\r");
										DesplazaFsmState = BACKWARD;
									}
									else
										if (strcmp(msg[0], "DERE") == 0)
										{
											printf ("Comando RIGHT \n\r");
											DesplazaFsmState = RIGHT;
										}
										else
											if (strcmp(msg[0], "IZQ") == 0)
											{
												printf ("Comando LEFT \n\r");
												DesplazaFsmState = LEFT;
											}
											else /* default: */
											{
												printf ("Comando Incorrecto \n\r");
												uartWriteString( UART_BLUETOOTH, "Comando incorrecto\r\n" );
											}
					}

			/*if( uartReadByte( UART_BLUETOOTH, &data ) ) {

				if( data == '0' ) {
					DesplazaFsmState = MOTORS_STOP;
					//motorControl (0, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				if( data == '1' ) {
					DesplazaFsmState = FORWARD;
					//motorControl (1, 10, 20);
					// gpioWrite( LEDB, OFF );
				}
				if( data == '2' ) {
					DesplazaFsmState = BACKWARD;
					//motorControl (2, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				if( data == '3' ) {
					DesplazaFsmState = RIGHT;
					//motorControl (3, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				if( data == '4' ) {
					DesplazaFsmState = LEFT;
					//motorControl (4, 10, 20);
					//gpioWrite( LEDB, ON );
				}
				// Esto me va a servir para reenviar data de la UART BLUETOOTH a la PC para DEBUG.
				uartWriteString( UART_PC, "Byte Recibido por BT: \r\n" );
				uartWriteByte( UART_PC, data );

			}*/
		}
	}
}

// Implementacion de funcion de la tarea
void motorControlTask ( void* taskParmPtr )
{
   //fsmButtonInit();

   // Tarea periodica cada 1 ms
   portTickType xPeriodicity =  20 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE)
   {
	   DesplazaFsmUpdate();

      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
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

void verifHC05 (void)
		{
	// Para entrar en modo AT en el HC-05. VERSION:2.0-20100601.
	   gpioWrite(CAN_TD,LOW);	// Apago HC-05 BT
	   gpioWrite(T_FIL0,HIGH);	// Pongo PIN 34 HIGH
	   gpioWrite(CAN_TD,HIGH);	// Enciendo HC-05 BT
	   vTaskDelay( 500 / portTICK_RATE_MS );
	   //delay(1000);	// Para darle tiempo a que el HC-05 reaccione.
	   uartConfig( UART_BLUETOOTH, 38400);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
	   //vTaskDelay( 500 / portTICK_RATE_MS );
	   //delay(1000);	// Para darle tiempo a que el HC-05 reaccione.
	   uartWriteString( UART_PC, "UART_BLUETOOTH Configurado para Modo AT (38400).\r\n" );
	   // Para entrar en modo AT en el HC-05.


	   // Verifico funcionamiento de modulo HC-05.
	   uartWriteString( UART_PC, "Testeo si el modulo esta conectado enviando: AT\r\n" );
	      if( hm10bleTest( UART_BLUETOOTH ) ){
	         uartWriteString( UART_PC, "Modulo conectado correctamente.\r\n" );
	      }
	      else{
	         uartWriteString( UART_PC, "No funciona.\r\n" );
	      }

	      // Verifico funcionamiento de modulo HC-05.

	   // Inicializar UART_232 para conectar al modulo bluetooth
	   //uartConfig( UART_BLUETOOTH, 38400);
	   //uartWriteString( UART_PC, "UART_BLUETOOTH para modulo Bluetooth configurada.\r\n" );


	   // Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
	      gpioWrite(CAN_TD,LOW);	// Apago HC-05 BT
	      gpioWrite(T_FIL0,LOW);	// Pongo PIN 34 LOW
	      delay(1000);	// Para darle tiempo a que el HC-05 reaccione.
	      gpioWrite(CAN_TD,HIGH);	// Enciendo HC-05 BT
	      uartConfig( UART_BLUETOOTH, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
	      delay(1000);	// Para darle tiempo a que el HC-05 reaccione.
	      uartWriteString( UART_PC, "UART_BLUETOOTH Configurado para Modo SSP (9600).\r\n" );
	      // Para salir de modo AT en el HC-05. VERSION:2.0-20100601.
		}


/*==================[fin del archivo]========================================*/
