#ifndef __BSP_BTIM_H
#define __BSP_BTIM_H

/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*定时器6相关宏*/
#define    Timx1                   TIM6
#define    Timx1_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define    Timx1_CLK               RCC_APB1Periph_TIM6
#define    TIMx1_IRQHandler        TIM6_IRQHandler
#define    Timx1_IRQn              TIM6_IRQn
/*定时器7相关宏*/
#define    Timx2                   TIM7
#define    Timx2_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define    Timx2_CLK               RCC_APB1Periph_TIM7
#define    TIMx2_IRQHandler        TIM7_IRQHandler
#define    Timx2_IRQn              TIM7_IRQn
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void TIM_Init(void);
/*------------------------------------test------------------------------------*/

#endif /*__BSP_BTIM_H*/
