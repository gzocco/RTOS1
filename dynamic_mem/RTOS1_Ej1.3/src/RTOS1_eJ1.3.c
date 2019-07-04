/* Gustavo Zocco
 * RTOS1 Ejercicio 1.3
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

// sAPI header
#include "sapi.h"
#//include "antiRebote.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/
// Declaraciones para la maquina de estados del anti rebote.
		typedef enum{
			BOTTON_UP,
			BOTTON_FALLING,
			BOTTON_DOWN,
			BOTTON_RAISING
		} fsmState_t;

		// Variable that hold the current state
		fsmState_t fsmState;
		//uint8_t valTecla=0;

		// FSM functions
		//void antiRebote( void );
		void fsmError( void );
		void fsmInit( void );
		void fsmUpdate( void );
		void bottonPressed (void);
		void bottonReleased (void);
		//void destelloLed (uint32_t tButtonDown);
	//%%%%%%%%%%%%%%%%%%%%


/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void myTask( void* taskParmPtr );

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );
   debugPrintlnString( "Blinky con freeRTOS y sAPI." );

   // Led para dar se�al de vida
  // gpioWrite( LED3, ON );

   // Crear tarea en freeRTOS
   xTaskCreate(
      myTask,                     // Funcion de la tarea a ejecutar
      (const char *)"myTask",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+1,         // Prioridad de la tarea
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

// Implementacion de funcion de la tarea
void myTask( void* taskParmPtr )
{
	//&&&&&&&&&&&&&&&&&&&&&&&&&&&7



			// Variable that hold the current state
			extern fsmState_t fsmState;
			//uint8_t valTecla=0;

			// FSM functions
			//void antiRebote( void );
			extern void fsmError( void );
			extern void fsmInit( void );
			extern void fsmUpdate( void );
			extern void bottonPressed (void);
			extern void bottonReleased (void);

			//&&&&&&&&&&&&&&&&&&&&&&
	//Inicializo maquina de estados.
	fsmInit();

	// Variable para cuenta de tiempo de tecla presionada.
	uint32_t tButtonDown=0;
	//static uint32_t tCount=0;
	static uint8_t debounceTime=20;
	//static uint16_t periodo=1000;

	// ---------- CONFIGURACIONES ------------------------------
  // printf( "Blinky con freeRTOS y sAPI.\r\n" );

 //  gpioWrite( LED1, ON );
   // Envia la tarea al estado bloqueado durante 1 s (delay)
 //  vTaskDelay( 1000 / portTICK_RATE_MS );
 //  gpioWrite( LED1, OFF );

   // Tarea periodica cada ciclo y periodo-ciclo en ms
   portTickType xPeriodicity =  debounceTime / portTICK_RATE_MS;
   //portTickType xPeriodicity2 =  ciclo / portTICK_RATE_MS;
   portTickType xPressTime = xTaskGetTickCount();
   portTickType xLastWakeTime = xTaskGetTickCount();
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
	   printf( "Actualizo fsm\r\n" );
	   //void fsmUpdate( void );
	   static bool_t flagUp = FALSE;
	static bool_t flagFalling = FALSE;
	static bool_t flagDown = FALSE;
	static bool_t flagRaising = FALSE;
	printf( "Actualizo fsm3\r\n" );
	switch( fsmState ){
	case BOTTON_UP:
		printf( "UP\r\n");
		if( flagUp == FALSE ){
			flagUp = TRUE;
		}
		if (!gpioRead(TEC1))
		{
			//flagFalling = TRUE;
			fsmState=BOTTON_FALLING;
			printf( "FALLING\r\n" );
		}
		if( fsmState != BOTTON_UP ){
			flagUp = FALSE;
		}
		// valTecla= !gpioRead (TEC1); ...
		break;

	case BOTTON_FALLING:
		if( flagFalling == FALSE ){
			flagFalling = TRUE;
		}
		if (!gpioRead(TEC1))
		{
			if (flagFalling == TRUE)
			{
				fsmState=BOTTON_DOWN;
				buttonPressed();
			}
		}
		else
		{
			flagFalling = FALSE;
			fsmState=BOTTON_UP;
		}
		if( fsmState != BOTTON_FALLING ){
			flagFalling = FALSE;
		}
		break;
	case BOTTON_DOWN:
		if( flagDown == FALSE ){
			flagDown = TRUE;
		}
		if (!gpioRead(TEC1))
		{
			flagDown = TRUE;
			fsmState=BOTTON_DOWN;
		}
		else
		{
			//if (flagDown == TRUE)
				// {
				fsmState=BOTTON_RAISING;
				//buttonReleased();
				// }
				//flagDown = FALSE;
		}
		if( fsmState != BOTTON_DOWN ){
			flagDown = FALSE;

		}
		break;

	case BOTTON_RAISING:
		if( flagRaising == FALSE ){
			flagRaising = TRUE;
		}
		if (gpioRead(TEC1))
		{
			if (flagRaising == TRUE)
			{
				buttonReleased();
				fsmState=BOTTON_UP;
			}
			// ...
		}
		if( fsmState != BOTTON_RAISING ){
			flagRaising = FALSE;
			// Code for leave...
		}
		break;

	default:
		fsmError();
		break;
	}



	   printf( "Actualizo fsm2\r\n" );
	   // Puede tener 20 ms de error en la medicion del tiempo que se mantiene presionado la tecla.
	   vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
	   // gpioToggle( LEDB );
      /*


	   gpioWrite (LEDB, HIGH);
      printf( "Blink ON!\r\n" );
      vTaskDelayUntil( &xLastWakeTime, xPeriodicity2 );
      gpioWrite (LEDB, LOW);
      printf( "Blink OFF!\r\n" );
      ciclo=ciclo+100;
      if (ciclo>=1000){
    	  ciclo=100;
      }

      xPeriodicity =  (periodo-ciclo) / portTICK_RATE_MS;
      xPeriodicity2 =  ciclo / portTICK_RATE_MS;
      //xLastWakeTime = xTaskGetTickCount();
       *
       */
   }
}

// Funciones %%%%%%%%%%%%%%%%

void fsmInit( void )
{
	// Example:
	// boardInit();          // Initialize hardware
	printf( "Inicio fsm\r\n");
	fsmState = BOTTON_UP;   // Set initial state
}


void buttonPressed( void )
{
	 portTickType xLastPressTime;
	printf( "Tecla presionada inicio cuenta de ticks.\r\n" );
	xLastPressTime = xTaskGetTickCount();
	//gpioWrite (LED1, ON);
}

void buttonReleased( void )
{
	portTickType xLastPressTime;
	portTickType tButtonDown;
	printf( "Tecla liberada resto ticks guardadas a ticks actuales para saber tiempo transcurrido.\r\n" );

	tButtonDown = xTaskGetTickCount() - xLastPressTime;
	printf( "Destello LED por %lu\r\n", &tButtonDown );
	//destelloLed(tButtonDown);

	gpioToggle (LED1);
}

void fsmError (void)
{
	printf( "FSM Error\r\n" );
}

/*
void destelloLed (tButtonDown)
{
	extern tButtonDown;
	gpioToggle(LEDB);
	vTaskDelayUntil( &xLastWakeTime, tButtonDown );
	gpioToggle(LEDB);
}
*/

// FSM Update Sate Function
void fsmUpdate( void )
{
	static bool_t flagUp = FALSE;
	static bool_t flagFalling = FALSE;
	static bool_t flagDown = FALSE;
	static bool_t flagRaising = FALSE;
	printf( "Actualizo fsm3\r\n" );
	switch( fsmState ){
	case BOTTON_UP:
		printf( "UP\r\n");
		if( flagUp == FALSE ){
			flagUp = TRUE;
		}
		if (!gpioRead(TEC1))
		{
			//flagFalling = TRUE;
			fsmState=BOTTON_FALLING;
			printf( "FALLING\r\n" );
		}
		if( fsmState != BOTTON_UP ){
			flagUp = FALSE;
		}
		// valTecla= !gpioRead (TEC1); ...
		break;

	case BOTTON_FALLING:
		if( flagFalling == FALSE ){
			flagFalling = TRUE;
		}
		if (!gpioRead(TEC1))
		{
			if (flagFalling == TRUE)
			{
				fsmState=BOTTON_DOWN;
				buttonPressed();
			}
		}
		else
		{
			flagFalling = FALSE;
			fsmState=BOTTON_UP;
		}
		if( fsmState != BOTTON_FALLING ){
			flagFalling = FALSE;
		}
		break;
	case BOTTON_DOWN:
		if( flagDown == FALSE ){
			flagDown = TRUE;
		}
		if (!gpioRead(TEC1))
		{
			flagDown = TRUE;
			fsmState=BOTTON_DOWN;
		}
		else
		{
			//if (flagDown == TRUE)
				// {
				fsmState=BOTTON_RAISING;
				//buttonReleased();
				// }
				//flagDown = FALSE;
		}
		if( fsmState != BOTTON_DOWN ){
			flagDown = FALSE;

		}
		break;

	case BOTTON_RAISING:
		if( flagRaising == FALSE ){
			flagRaising = TRUE;
		}
		if (gpioRead(TEC1))
		{
			if (flagRaising == TRUE)
			{
				buttonReleased();
				fsmState=BOTTON_UP;
			}
			// ...
		}
		if( fsmState != BOTTON_RAISING ){
			flagRaising = FALSE;
			// Code for leave...
		}
		break;

	default:
		fsmError();
		break;
	}
}

/*==================[fin del archivo]========================================*/
