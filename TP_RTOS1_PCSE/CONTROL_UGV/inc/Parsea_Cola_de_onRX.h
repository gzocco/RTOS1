/*
 * Parsea_Cola_de_onRX.h
 *
 *  Created on: Aug 15, 2019
 *      Author: gzocco
 */

#ifndef PROGRAMS_RTOS1_TP_RTOS1_PCSE_CONTROL_UGV_INC_PARSEA_COLA_DE_ONRX_H_
#define PROGRAMS_RTOS1_TP_RTOS1_PCSE_CONTROL_UGV_INC_PARSEA_COLA_DE_ONRX_H_
//Para recepciones de frame por UART BT.
#define MAX_MSG_SIZE 30		// "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx\n" string del mensaje esperado
							// Ver de usar una struct y calcular el tamaño con sizeof...
#define SEPARADOR ","
void Parsea_Cola_de_onRX( void* pvParameters );

#endif /* PROGRAMS_RTOS1_TP_RTOS1_PCSE_CONTROL_UGV_INC_PARSEA_COLA_DE_ONRX_H_ */
