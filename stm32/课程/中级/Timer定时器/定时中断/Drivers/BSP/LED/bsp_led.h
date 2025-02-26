#ifndef _BSP_LED_H_
#define _BSP_LED_H_

#include "stm32f10x.h"

/**
 *   GPIO相关宏定义
 */
#define LED_GPIO_APBxClock_FUN RCC_APB2PeriphClockCmd
#define LED_GPIO_CLK RCC_APB2Periph_GPIOB
#define LED_R_GPIO_PORT GPIOB
#define LED_R_GPIO_Pin GPIO_Pin_5
#define LED_G_GPIO_PORT GPIOB
#define LED_G_GPIO_Pin GPIO_Pin_0
#define LED_B_GPIO_PORT GPIOB
#define LED_B_GPIO_Pin GPIO_Pin_1

#define ON 1
#define OFF 0

#define LED_OFF GPIO_SetBits(LED_B_GPIO_PORT, LED_B_GPIO_Pin), \
                GPIO_SetBits(LED_G_GPIO_PORT, LED_G_GPIO_Pin), \
                GPIO_SetBits(LED_R_GPIO_PORT, LED_R_GPIO_Pin)

/**
 *  开关LED
 */
#define LEDR_ON GPIO_ResetBits(LED_R_GPIO_PORT, LED_R_GPIO_Pin);
#define LEDR_OFF GPIO_SetBits(LED_R_GPIO_PORT, LED_R_GPIO_Pin);

#define LEDG_ON GPIO_ResetBits(LED_G_GPIO_PORT, LED_G_GPIO_Pin);
#define LEDG_OFF GPIO_SetBits(LED_G_GPIO_PORT, LED_G_GPIO_Pin);

#define LEDB_ON GPIO_ResetBits(LED_B_GPIO_PORT, LED_B_GPIO_Pin);
#define LEDB_OFF GPIO_SetBits(LED_B_GPIO_PORT, LED_B_GPIO_Pin);

#define LED_OFF GPIO_SetBits(LED_B_GPIO_PORT, LED_B_GPIO_Pin), \
                GPIO_SetBits(LED_G_GPIO_PORT, LED_G_GPIO_Pin), \
                GPIO_SetBits(LED_R_GPIO_PORT, LED_R_GPIO_Pin)

/**
 *  翻转LED
 */
#define LEDG_TOGGLE LED_G_GPIO_PORT->ODR ^= LED_G_GPIO_Pin
#define LEDR_TOGGLE LED_R_GPIO_PORT->ODR ^= LED_R_GPIO_Pin
#define LEDB_TOGGLE LED_B_GPIO_PORT->ODR ^= LED_B_GPIO_Pin

void LED_Init(void);

#endif /*_BSP_LED_H_*/
