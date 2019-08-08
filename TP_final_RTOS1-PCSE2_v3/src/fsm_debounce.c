/* Copyright 2018, Eric Pernia.
 * All rights reserved.
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
 *
 */
 
//#include "../../btControl.h"
#include "accionaMotores.h"
#include "sapi.h"

typedef enum{
   STATE_BUTTON_UP,
   STATE_BUTTON_DOWN,
   STATE_BUTTON_FALLING,
   STATE_BUTTON_RISING
} fsmButtonState_t;

void fsmButtonError( void );
void fsmButtonInit( void );
void fsmButtonUpdate( gpioMap_t tecla );
void buttonPressed( void );
void buttonReleased( void );

fsmButtonState_t fsmButtonState;

uint32_t t_pulsacion = 0;
extern uint32_t counter;

extern DesplazaFsmState_t DesplazaFsmState;

uint32_t get_t_pulsacion()
{
	return t_pulsacion;
}

void  reset_t_pulsacion()
{
	  t_pulsacion = 0 ;
}

void buttonPressed( void )
{

	static uint8_t cont = 0;
   gpioWrite(LEDR, OFF);
   gpioWrite(LEDG, OFF);
   gpioWrite(LEDB, ON);
   //debugPrintlnString( "buttonPressed" );
   uartWriteString( UART_PC, "buttonPressed.\r\n" );
   DesplazaFsmState = FORWARD;
   if(cont>5){
      gpioWrite(LEDR, OFF);
      gpioWrite(LEDG, ON);
      gpioWrite(LEDB, OFF);
   }
   cont++;
}

void buttonReleased( void )
{
   static uint8_t cont = 0;
   gpioWrite(LEDR, ON);
   gpioWrite(LEDG, OFF);
   gpioWrite(LEDB, OFF);
   uartWriteString( UART_PC, "buttonReleased.\r\n" );
      DesplazaFsmState = MOTORS_STOP;
   if(cont>10){
      gpioWrite(LEDR, OFF);
      gpioWrite(LEDG, OFF);
      gpioWrite(LEDB, OFF);
   }
   cont++;
}

void fsmButtonError( void )
{
   fsmButtonState = BUTTON_UP;
}

void fsmButtonInit( void )
{
   fsmButtonState = BUTTON_UP;  // Set initial state
}

// FSM Update Sate Function
void fsmButtonUpdate( gpioMap_t tecla )
{
   static bool_t flagFalling = FALSE;
   static bool_t flagRising = FALSE;
   
   static uint8_t contFalling = 0;
   static uint8_t contRising = 0;

   static uint32_t t_ini; //deberian ser static??
   static uint32_t t_fin;

   //counter ++; // No va porque lo incremento con void vApplicationTickHook( void )
  // {
   //	counter++;
//   }

   switch( fsmButtonState )
   {
      case STATE_BUTTON_UP: 
         /* CHECK TRANSITION CONDITIONS */
         if( !gpioRead(tecla) ){
            fsmButtonState = STATE_BUTTON_FALLING;
         }
      break;

      case STATE_BUTTON_DOWN:
         /* CHECK TRANSITION CONDITIONS */
         if( gpioRead(tecla) ){
            fsmButtonState = STATE_BUTTON_RISING;
         }
      break;

      case STATE_BUTTON_FALLING:      
         /* ENTRY */
         if( flagFalling == FALSE ){
            flagFalling = TRUE;

         }      
         /* CHECK TRANSITION CONDITIONS */
         if( contFalling >= 40 ){
            if( !gpioRead(tecla) ){
               fsmButtonState = STATE_BUTTON_DOWN;
              buttonPressed();
               t_ini=counter;
            } else{
               fsmButtonState = STATE_BUTTON_UP;
            }
            contFalling = 0;
         }
         contFalling++;
         /* LEAVE */
         if( fsmButtonState != STATE_BUTTON_FALLING ){
            flagFalling = FALSE;
         }
      break;

      case STATE_BUTTON_RISING:      
         /* ENTRY */
         if( flagRising == FALSE ){
            flagRising = TRUE;
         }    
         /* CHECK TRANSITION CONDITIONS */
         
         if( contRising >= 40 ){
            if( gpioRead(tecla) ){
               fsmButtonState = STATE_BUTTON_UP;
               buttonReleased();
               t_fin=counter;

               t_pulsacion = t_fin - t_ini;

            } else{
               fsmButtonState = STATE_BUTTON_DOWN;
            }
            contRising = 0;
         }
         contRising++;
         
         /* LEAVE */
         if( fsmButtonState != STATE_BUTTON_RISING ){
            flagRising = FALSE;
         }
      break;

      default:
         fsmButtonError();
      break;
   }
}
