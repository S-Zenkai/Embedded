
#ifndef _BSP_KEY_H
#define _BSP_KEY_H
#include "stm32f10x.h"
#define KEY1_GPIO_Port GPIOA
#define KEY2_GPIO_Port GPIOC
#define KEY1_GPIO_Pin GPIO_Pin_0
#define KEY2_GPIO_Pin GPIO_Pin_13
#define KEY1_GPIO_CLK RCC_APB2Periph_GPIOA
#define KEY2_GPIO_CLK RCC_APB2Periph_GPIOC


uint8_t KEY_CHECK(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

void KEY1_Config(void);
void KEY2_Config(void);

//按键按下高电平
#define ON 1
#define OFF 0

#endif /*_BSP_KEY_H*/

