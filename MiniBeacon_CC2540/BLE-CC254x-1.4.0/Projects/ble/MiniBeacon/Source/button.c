/*

  Copyright (c) 2013 RedBearLab

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal 
  in the Software without restriction, including without limitation the rights 
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "osal.h"
#include "MiniBeacon.h"

extern uint8 MiniBeacon_TaskID;

void ButtonInit(void)
{
    /* Key Init*/
  P1SEL &= 0xFB; //P1.2 is GPIO
  P1DIR &= 0xFB; //P1.2 is Input
  PICTL |= 0x02; //P1.2 Falling Edge give interrupt
  P1IEN |= 0x04; //P1.2 Enable Interrupt
  
  IEN2 |= 0x10;
  
  P0SEL &= 0x7F; //P0.7 is GPIO
  P0DIR |= 0x80; //P0.7 is Output
  P0_7 = 1;
}


/**************************************************************************************************
 * @fn      halKeyPort1Isr
 *
 * @brief   Port1 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( ButtonDownIsr, P1INT_VECTOR )
{
  HAL_ENTER_ISR();
  

  if(P1IFG & 0x04)
  {
    P1IFG &= 0xFB; //clear interrupt flag
    IRCON2 &= 0xF7; //clear CPU interrupt flag
    osal_start_timerEx (MiniBeacon_TaskID, SBP_BUTTON_DOWN_EVT, 25);

  }

  CLEAR_SLEEP_MODE();

  HAL_EXIT_ISR();

  return;
}