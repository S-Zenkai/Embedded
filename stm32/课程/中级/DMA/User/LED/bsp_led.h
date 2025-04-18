#ifndef _BSP_LED_H_
#define _BSP_LED_H_

#include "stm32f10x.h"

#define LED_G_GPIO_CLK     RCC_APB2Periph_GPIOB
#define LED_G_GPIO_PORT    GPIOB
#define LED_G_GPIO_Pin     GPIO_Pin_0

#define LED_B_GPIO_CLK     RCC_APB2Periph_GPIOB
#define LED_B_GPIO_PORT    GPIOB
#define LED_B_GPIO_Pin     GPIO_Pin_1

#define LED_R_GPIO_CLK     RCC_APB2Periph_GPIOB
#define LED_R_GPIO_PORT    GPIOB
#define LED_R_GPIO_Pin     GPIO_Pin_5

void LED_G_Config(void);
void LED_R_Config(void);
void LED_B_Config(void);


//#define ON 1
//#define OFF 0

#if 0
#define LED_G(a)       if(a==ON)\
	                                GPIO_ResetBits(LED_G_GPIO_PORT, LED_G_GPIO_Pin);\
                               else\
                                   GPIO_SetBits (LED_G_GPIO_PORT, LED_G_GPIO_Pin)

#define LED_R(a)       if(a==ON)\
	                                GPIO_ResetBits(LED_R_GPIO_PORT, LED_R_GPIO_Pin);\
                               else\
                                   GPIO_SetBits (LED_R_GPIO_PORT, LED_R_GPIO_Pin)

#define LED_B(a)       if(a==ON)\
	                                GPIO_ResetBits(LED_B_GPIO_PORT, LED_B_GPIO_Pin);\
                               else\
                                   GPIO_SetBits (LED_B_GPIO_PORT, LED_B_GPIO_Pin)
#endif


#define LED_OFF         GPIO_SetBits(GPIOB , GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5)

#define LED_R_ON        GPIO_ResetBits(LED_R_GPIO_PORT , LED_R_GPIO_Pin)
#define LED_R_OFF       GPIO_SetBits(LED_R_GPIO_PORT , LED_R_GPIO_Pin)

#define LED_G_ON        GPIO_ResetBits(LED_G_GPIO_PORT , LED_G_GPIO_Pin)
#define LED_G_OFF       GPIO_SetBits(LED_G_GPIO_PORT , LED_G_GPIO_Pin)

#define LED_B_ON        GPIO_ResetBits(LED_B_GPIO_PORT , LED_B_GPIO_Pin)
#define LED_B_OFF       GPIO_SetBits(LED_B_GPIO_PORT , LED_B_GPIO_Pin)

#define LED_Y_ON        GPIO_ResetBits(LED_R_GPIO_PORT , LED_R_GPIO_Pin|LED_G_GPIO_Pin)
#define LED_Y_OFF       GPIO_SetBits(LED_R_GPIO_PORT , LED_R_GPIO_Pin|LED_G_GPIO_Pin)

#define LED_G_TOGGLE    LED_G_GPIO_PORT->ODR ^= LED_G_GPIO_Pin
#define LED_R_TOGGLE    LED_R_GPIO_PORT->ODR ^= LED_R_GPIO_Pin
#define LED_B_TOGGLE    LED_B_GPIO_PORT->ODR ^= LED_B_GPIO_Pin



void LED_Config(void);

#endif /*_BSP_LED_H_*/

