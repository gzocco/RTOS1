/* Gustavo Zocco
 * RTOS1 Ejercicio 1.2
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

// sAPI header
#include "sapi.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

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
   static uint16_t ciclo=100;
   static uint16_t periodo=1000;
	// ---------- CONFIGURACIONES ------------------------------
  // printf( "Blinky con freeRTOS y sAPI.\r\n" );

 //  gpioWrite( LED1, ON );
   // Envia la tarea al estado bloqueado durante 1 s (delay)
 //  vTaskDelay( 1000 / portTICK_RATE_MS );
 //  gpioWrite( LED1, OFF );

   // Tarea periodica cada ciclo y periodo-ciclo en ms
   portTickType xPeriodicity =  (periodo-ciclo) / portTICK_RATE_MS;
   portTickType xPeriodicity2 =  ciclo / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();
   
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
	   // gpioToggle( LEDB );
      gpioWrite (LEDB, HIGH);
      printf( "Blink ON!\r\n" );
      vTaskDelayUntil( &xLastWakeTime, xPeriodicity2 );
      gpioWrite (LEDB, LOW);
      printf( "Blink OFF!\r\n" );
      ciclo=ciclo+100;
      if (ciclo>=1000){
    	  ciclo=100;
      }
      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
      xPeriodicity =  (periodo-ciclo) / portTICK_RATE_MS;
      xPeriodicity2 =  ciclo / portTICK_RATE_MS;
      //xLastWakeTime = xTaskGetTickCount();
   }
}

/*==================[fin del archivo]========================================*/
