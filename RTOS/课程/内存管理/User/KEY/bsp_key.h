#ifndef __BSP_KEY_H
#define __BSP_KEY_H
#include "stm32f10x.h"

//GPIO相关宏定义
#define     KEY_GPIO_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define     KEY_GPIO_CLC                 RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC
#define     KEY1_GPIO_PIN                GPIO_Pin_0
#define     KEY1_GPIO_PORT               GPIOA
#define     KEY2_GPIO_PIN                GPIO_Pin_13
#define     KEY2_GPIO_PORT               GPIOC

//按键状态
#define     KEY1_ON                       1
#define     KEY1_OFF                      0
#define     KEY2_ON                       1
#define     KEY2_OFF                      0

#define     KEY1                          1
#define     KEY2                          2


void KEY_Init(void);
uint8_t Get_Keyx_Status(uint8_t val);

#endif /*__BSP_KEY_H*/

