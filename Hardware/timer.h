
#ifndef __TIMER_H
#define __TIMER_H
/******************************************************************************/
#include "stm32f4xx.h"
/******************************************************************************/


/******************************************************************************/
void TIM1_PWM_Init(void);
void TIM2_Init(void);
void TIM3_Encoder_Init(void);
void TIM3_InputCapture_Config(void);
void TIM7_Init(void);
extern uint64_t can_cnt;
extern float temperature_motor, temperature_board;
/******************************************************************************/

#endif
