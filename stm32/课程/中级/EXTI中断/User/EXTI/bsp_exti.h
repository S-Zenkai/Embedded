#ifndef _BSP_EXTI_H
#define _BSP_EXTI_H
#include "stm32f10x.h"

//按键端口和位
#define KEY1_GPIO_Port GPIOA
#define KEY2_GPIO_Port GPIOC
#define KEY1_GPIO_Pin GPIO_Pin_0
#define KEY2_GPIO_Pin GPIO_Pin_13
//按键时钟
#define KEY1_GPIO_CLK RCC_APB2Periph_GPIOA
#define KEY2_GPIO_CLK RCC_APB2Periph_GPIOC

void KEY_EXTI_Config(void);
void NVIC_EXTI_Config(void);


#endif /*_BSP_EXTI_H*/
