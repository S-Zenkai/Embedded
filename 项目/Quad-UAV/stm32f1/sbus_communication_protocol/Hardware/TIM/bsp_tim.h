#ifndef __BSP_TIM_H 
#define __BSP_TIM_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*TIM����Ƚ����GPIO��*/
#define                 PWMx_TIM_GPIO_CLK_FUN           RCC_APB2PeriphClockCmd
#define                 PWMx_TIM_GPIO_CLK               RCC_APB2Periph_GPIOA
#define                 PWMA_TIM_GPIO_Pin               GPIO_Pin_6
#define                 PWMA_TIM_GPIO_Port              GPIOA
#define                 PWMB_TIM_GPIO_Pin               GPIO_Pin_7
#define                 PWMB_TIM_GPIO_Port              GPIOA
/*TIM��غ궨��*/
#define                 Timx                            TIM3
#define                 PWMx_TIM_CLK_FUN                RCC_APB1PeriphClockCmd
#define                 PWMx_TIM_CLK                    RCC_APB1Periph_TIM3
#define                 PWMA_TIM_OCInitFUN              TIM_OC1Init
#define                 PWMB_TIM_OCInitFUN              TIM_OC2Init
/*----------------------------------function----------------------------------*/
void timer_init(void);
uint64_t timer_GetTick(void);
/*------------------------------------test------------------------------------*/

#endif	/* __BSP_TIM_H */



