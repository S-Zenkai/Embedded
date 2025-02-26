#ifndef _BSP_KEY_H_
#define _BSP_KEY_H_
#include "stm32f10x.h"

#define KEY1_CLK   RCC_APB2Periph_GPIOA
#define KEY2_CLK   RCC_APB2Periph_GPIOC

#define KEY1_Pin   GPIO_Pin_0
#define KEY2_Pin   GPIO_Pin_13

#define KEY1_Port    GPIOA
#define KEY2_Port    GPIOC

void KEY1_Config(void);
void KEY2_Config(void);

#define KEY1_ON 1
#define KEY1_OFF 0
#define KEY2_ON 1
#define KEY2_OFF 0


#endif /*_BSP_KEY_H_*/

