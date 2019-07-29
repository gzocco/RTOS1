/*==============================================================================
 * Author: Gustavo Zocco <gzocco@gmail.com>
 * Date: 2019/06/02
 * Version: 0.1
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/
#include "sapi.h"
#include "btControl.h"
#include "btInit.h"
#include "motorControl.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
// FSM DECLARATIONS
// FSM state names
/*
typedef enum{
	FORWARD,
	BACKWARD,
	RIGHT,
	LEFT,
	MOTORS_STOP
} DesplazaFsmState_t;
*/

// Variable that hold the current state
DesplazaFsmState_t DesplazaFsmState;

// FSM functions
void DesplazaFsmError( void );
void DesplazaFsmUpdate( void );
void DesplazaFsmInit( void );

/*=====[Main function, program entry point after power on or reset]==========*/

int pepe( void )
{
	// ----- Setup -----------------------------------
	btInit ();
	DesplazaFsmInit();
	uint8_t data = 0;
	while( true ) {
		if( delayRead(&tiempo1) ) {
			DesplazaFsmUpdate();
		}
		if( uartReadByte( UART_BLUETOOTH, &data ) ) {
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
			//uartWriteByte( UART_PC, data );
		}
		// ----- Repeat for ever -------------------------
		//gpioToggle(LED);
		//delay(500);
	}
	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}

/*==================[definiciones de funciones externas]=====================*/

bool_t hm10bleTest( int32_t uart )
{
   uartWriteString( uart, "AT\r\n" );
   return waitForReceiveStringOrTimeoutBlocking( uart,
                                                 "OK\r\n", strlen("OK\r\n"),
                                                 1000 );
}
/*
void hm10blePrintATCommands( int32_t uart )
{
   delay(500);
   uartWriteString( uart, "AT+HELP\r\n" );
}
*/

// FSM Error Handler Function
void DesplazaFsmError( void )
{
	// Error handler, example, restart FSM:
	DesplazaFsmState = MOTORS_STOP;
}

// FSM Initialize Function
void DesplazaFsmInit( void )
{
	// Example:
	// boardInit();          // Initialize hardware
	DesplazaFsmState = MOTORS_STOP;   // Set initial state
}

// FSM Update Sate Function
void DesplazaFsmUpdate( void )
{
	switch(DesplazaFsmState)
	{
	case MOTORS_STOP:
		motorControl(0,10,10);
		break;
	case FORWARD:
		motorControl(1,10,10);
		break;
	case BACKWARD:
		motorControl(2,10,10);
		break;
	case RIGHT:
		motorControl(3,10,10);
		break;
	case LEFT:
		motorControl(4,10,10);
		break;
	default:
		DesplazaFsmError();
		break;
	}
}
