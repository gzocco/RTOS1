/*
 * btInit.h
 *
 *  Created on: Jun 18, 2019
 *      Author: gzocco
 */

#ifndef PDM_BTCONTROL_SRC_BTINIT_H_
#define PDM_BTCONTROL_SRC_BTINIT_H_
#include <stdint.h>
#include <stddef.h>
#include <string.h>
/*=====[Definition macros of public constants]===============================*/
#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232
/*=====[Public function-like macros]=========================================*/
bool_t hm10bleTest( int32_t uart );
void hm10blePrintATCommands( int32_t uart );
void btInit(void);
#endif /* PDM_BTCONTROL_SRC_BTINIT_H_ */
