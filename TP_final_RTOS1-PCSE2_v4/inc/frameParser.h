/*
 * frameParser.h
 *
 *  Created on: Aug 8, 2019
 *      Author: gzocco
 */

#ifndef PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V4_INC_FRAMEPARSER_H_
#define PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V4_INC_FRAMEPARSER_H_

//Para recepciones de frame por UART BT.
#define MAX_MSG_SIZE 30		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
							// Ver de usar una struct y calcular el tamaño con sizeof...
#define SEPARADOR ","

typedef struct {
	char cmd[6][5];		// Quitar luego de cambiar funcion parse.
	char cmd0[5];
	char cmd1[5];
	char cmd2[5];
	char cmd3[5];
	char cmd4[5];
	char cmd5[5];
} msg_t;

void frameParser (void);

typedef struct {
	uint8_t rxByte;		// Quitar luego de cambiar funcion parse.
} onRxByte_t;

extern QueueHandle_t xQueue_BTrx_frameParse;
extern QueueHandle_t xQueueBTrx_Control;

extern bool_t frame_rec;

#endif /* PROGRAMS_RTOS1_TP_FINAL_RTOS1_PCSE2_V4_INC_FRAMEPARSER_H_ */
