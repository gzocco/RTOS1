/*=============================================================================
 * Author: Gustavo Zocco <gzocco@gmail.com>
 * Date: 2019/06/02
 * Version: 0.1
 *===========================================================================*/

/*=====[Avoid multiple inclusion - begin]====================================*/

#ifndef __BTCONTROL_H__
#define __BTCONTROL_H__

/*=====[Inclusions of public function dependencies]==========================*/

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/*=====[Definition macros of public constants]===============================*/
#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232
/*=====[Public function-like macros]=========================================*/
bool_t hm10bleTest( int32_t uart );
void hm10blePrintATCommands( int32_t uart );
//Variable para temporizador ticks 1.
delay_t tiempo1;

typedef enum{
	FORWARD,
	BACKWARD,
	RIGHT,
	LEFT,
	MOTORS_STOP
} DesplazaFsmState_t;


/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif


/*=====[Definitions of public data types]====================================*/

/*=====[Prototypes (declarations) of public functions]=======================*/

/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* __BTCONTROL_H__ */
