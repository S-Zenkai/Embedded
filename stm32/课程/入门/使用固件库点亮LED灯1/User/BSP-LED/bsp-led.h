#ifndef _BSP_LED_H_
#define _BSP_LED_H_
#include "stm32f10x.h"
//#include "stm32f10x_gpio.h"

#define LED_G_Pin   GPIO_Pin_0
#define LED_B_Pin   GPIO_Pin_1
#define LED_R_Pin   GPIO_Pin_5

#define LED_G_CLK   RCC_APB2Periph_GPIOB
#define LED_B_CLK   RCC_APB2Periph_GPIOB
#define LED_R_CLK   RCC_APB2Periph_GPIOB

#define LED_G_Port    GPIOB
#define LED_B_Port    GPIOB
#define LED_R_Port    GPIOB



void LED_G_Config(void);
void LED_B_Config(void);
void LED_R_Config(void);

#define LED_G_OFF    GPIO_SetBits(LED_G_Port, LED_G_Pin)
#define LED_G_ON     GPIO_ResetBits(LED_G_Port, LED_G_Pin)
#define LED_B_OFF    GPIO_SetBits(LED_B_Port, LED_B_Pin)
#define LED_B_ON     GPIO_ResetBits(LED_B_Port, LED_B_Pin)
#define LED_R_OFF    GPIO_SetBits(LED_R_Port, LED_R_Pin)
#define LED_R_ON     GPIO_ResetBits(LED_R_Port, LED_R_Pin)
#define LED_G_TOGGLE     GPIOB->ODR ^= (1<<0)
#define LED_B_TOGGLE     GPIOB->ODR ^= (1<<1)
#define LED_R_TOGGLE     GPIOB->ODR ^= (1<<5)


#endif /*_BSP_LED_H_*/


