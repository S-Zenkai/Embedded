#ifndef __BSP_TIM_H
#define __BSP_TIM_H
#include "stm32f10x.h"

#define    Timx_APBxClock_FUN      RCC_APB1PeriphClockCmd
#define    Timx_CLK                RCC_APB1Periph_TIM7
#define    Timx_IRQn               TIM7_IRQn
#define    Timx                    TIM1

    void
    Timer_Init(void);

#endif /*__BSP_TIM_H*/
