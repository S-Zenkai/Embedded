#ifndef __BSP_PWM_H
#define __BSP_PWM_H
#include "stm32f10x.h"



/*宏定义*/

/*是否开启中断，0：不开启。1：开启*/
#define            InterruptConfiguration               0

#define            PWM_Timx_APBxClock_FUN               RCC_APB1PeriphClockCmd
#define            PWM_Timx_CLK                         RCC_APB1Periph_TIM3
#define            PWM_Timx                             TIM3

#define            PWM_GPIO_APBxClock_FUN               RCC_APB2PeriphClockCmd 
#define            PWM_GPIO_CLK                         RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO
#define            PWM_GPIO_Port                        GPIOB

#define            PWM_GPIOG_Pin                         GPIO_Pin_0
#define            PWM_GPIOR_Pin                         GPIO_Pin_5
#define            PWM_GPIOB_Pin                         GPIO_Pin_1



void TimOCInit(void);
#endif /*__BSP_PWM_H*/
