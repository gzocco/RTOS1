/* Copyright 2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
/*==================[inlcusiones]============================================*/

#include "sapi.h"        // <= Biblioteca sAPI
#include <string.h>

/*==================[definiciones y macros]==================================*/

#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

bool_t hm10bleTest( int32_t uart );
void hm10blePrintATCommands( int32_t uart );

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{
   // ---------- CONFIGURACIONES ------------------------------

   // Inicializar y configurar la plataforma
   boardConfig();


   gpioInit( CAN_RD, GPIO_INPUT );	// Pin STATE del modulo BT HC-05.
   gpioInit( CAN_TD, GPIO_OUTPUT );	// Transistor ON/OFF BT HC-05.
   gpioInit( T_FIL0, GPIO_OUTPUT );	// PIN 34 de BT HC-05.

   // Inicializar UART_USB para conectar a la PC
       uartConfig( UART_PC, 9600 );
       uartWriteString( UART_PC, "UART_PC configurada.\r\n" );


   // Para entrar en modo AT en el HC-05. VERSION:2.0-20100601.
   gpioWrite(CAN_TD,LOW);	// Apago HC-05 BT
   gpioWrite(T_FIL0,HIGH);	// Pongo PIN 34 HIGH
   gpioWrite(CAN_TD,HIGH);	// Enciendo HC-05 BT
   uartConfig( UART_BLUETOOTH, 38400);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
   delay(500);	// Para darle tiempo a que el HC-05 reaccione.
   uartWriteString( UART_PC, "UART_BLUETOOTH Configurado para Modo AT (38400).\r\n" );
   // Para entrar en modo AT en el HC-05.


   // Verifico funcionamiento de modulo HC-05.
   uartWriteString( UART_PC, "Testeto si el modulo esta conectado enviando: AT\r\n" );
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
      gpioWrite(T_FIL0,LOW);	// Pongo PIN 34 HIGH
      delay(500);	// Para darle tiempo a que el HC-05 reaccione.
      gpioWrite(CAN_TD,HIGH);	// Enciendo HC-05 BT
      uartConfig( UART_BLUETOOTH, 9600);	// Config UART a 38400 que es la BaudRate FIJO de la HC-05 en modo AT.
      delay(500);	// Para darle tiempo a que el HC-05 reaccione.
      uartWriteString( UART_PC, "UART_BLUETOOTH Configurado para Modo SSP (9600).\r\n" );
      // Para salir de modo AT en el HC-05. VERSION:2.0-20100601.


   uint8_t data = 0;
   


   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {

      // Si leo un dato de una UART lo envio a al otra (bridge)
	   if( uartReadByte( UART_PC, &data ) ) {
         uartWriteByte( UART_BLUETOOTH, data );
      }
     /* if (!gpioRead (CAN_RD)){
    	  uartWriteString( UART_PC, "Nadie conectado al BT.\r\n" );
      }*/

	   if( uartReadByte( UART_BLUETOOTH, &data ) ) {
         if( data == 'h' ) {
            gpioWrite( LEDB, ON );
         }
         if( data == 'l' ) {
            gpioWrite( LEDB, OFF );
         }
         uartWriteByte( UART_PC, data );
      }
      
      // Si presiono TEC1 imprime la lista de comandos AT
      if( !gpioRead( TEC1 ) ) {
         hm10blePrintATCommands( UART_BLUETOOTH );
      }
      
      // Si presiono TEC3 enciende el led de la pantalla de la app
      if( !gpioRead( TEC3 ) ) {
         uartWriteString( UART_BLUETOOTH, "LED_ON\r\n" );
         delay(500);
      }
      // Si presiono TEC4 apaga el led de la pantalla de la app
      if( !gpioRead( TEC4 ) ) {
         uartWriteString( UART_BLUETOOTH, "LED_OFF\r\n" );
         delay(500);
      }
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

bool_t hm10bleTest( int32_t uart )
{
	uartWriteString( uart, "AT\r\n" );	//Para HC-05
	//uartWriteString( uart, "AT" );
	//uartWriteString( uart, "" );//Para HC-06, van sin ENTER
	return waitForReceiveStringOrTimeoutBlocking( uart,
			"OK\r\n", strlen("OK\r\n"),
			1000 );
	/* return waitForReceiveStringOrTimeoutBlocking( uart,
                                                 "OK", strlen("OK"),
                                                 1000 );	//Para HC-06, van sin ENTER*/
}

void hm10blePrintATCommands( int32_t uart )
{
   delay(500);
   uartWriteString( uart, "AT+HELP\r\n" );
}

/*==================[fin del archivo]========================================*/
