/*=============================================================================
 * Author: Gustavo Zocco <gzocco@gmail.com>
 * Date: 2019/05/26
 * Version: 0.1
 *
 *
 *
 * PINES A SER UTILIZADOS PARA CONTROLAR LOS MOTORES CON PWM:
 *
 * MOTOR2:
 * EnB     P2.40    PWM6   GPIO8    (CTOUT7)
 * In3     P2.36           GPIO5
 * In4     P2.38           GPIO7

 * MOTOR1:
 * EnA     P2.31    PWM10  GPIO2    (CTOUT6)
 * In1     P2.33           GPIO4
 * In2     P2.35 o 29       GPIO6 o GPIO0

 * GND     P2.39 y P2.37
 *
 *
 *
 *===========================================================================*/

/*=====[Avoid multiple inclusion - begin]====================================*/
#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__
// Defino los pines que controlan los motores.
#define motor1EnA	GPIO2	//PWM10	//LED1	//Deberian tener capacidad de PWM
#define motor1Ini1	GPIO4	//LED2
#define motor1Ini2	GPIO0	//LED3
#define motor2EnB	GPIO8	//PWM6	//LEDR	//Deberian tener capacidad de PWM
#define motor2Ini3	GPIO5	//LEDG	//GPIO4
#define motor2Ini4	GPIO7	//LEDB

// Funcion para controller el accionamiento de los motores (F,Rev, Rig, Left, stop. Y la velocidad.)
int motorControl (uint8_t direction, uint8_t speedMotor1, uint8_t speedMotor2);
/*=====[Inclusions of public function dependencies]==========================*/
#include <stdint.h>
#include <stddef.h>


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

#endif /* __MOTORCONTROL_H__ */
