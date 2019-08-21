/*=============================================================================
 * Author: Gustavo Zocco <gzocco@gmail.com>
 * Date: 2019/08/15
 * Version: 0.1
 *===========================================================================*/

/*=====[Avoid multiple inclusion - begin]====================================*/

#ifndef __CONTROL_UGV_H__
#define __CONTROL_UGV_H__

/*=====[Inclusions of public function dependencies]==========================*/

#include <stdint.h>
#include <stddef.h>
#include "queue.h"
#include "task.h"

typedef struct {
	char cmd0[5];
	char cmd1[5];
	char cmd2[5];
	char cmd3[5];
	char cmd4[5];
	char cmd5[5];
} msg_t;

QueueHandle_t xQueue_BTrx_frameParser;
QueueHandle_t xQueue_frameParser_Control;

TaskHandle_t xHandle_Parsea_Cola_de_onRX;
TaskHandle_t xHandle_Recibe_Cola_de_onRX;

/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/

/*=====[Public function-like macros]=========================================*/

/*=====[Definitions of public data types]====================================*/

/*=====[Prototypes (declarations) of public functions]=======================*/

/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* __CONTROL_UGV_H__ */
