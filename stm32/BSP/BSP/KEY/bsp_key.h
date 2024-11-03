#ifndef __BSP_KEY_H
#define	__BSP_KEY_H
#include "stm32f10x.h"


//  Òý½Å¶¨Òå
#define    KEY_GPIO_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define    KEY_GPIO_CLK                RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC
#define	   KEY1_GPIO_PORT              GPIOA
#define  	 KEY1_GPIO_Pin 			    	   GPIO_Pin_0
#define	   KEY2_GPIO_PORT              GPIOC
#define  	 KEY2_GPIO_Pin 			    	   GPIO_Pin_13



void KEY_Init(void);
uint8_t Key_CheckPress(uint8_t val);



#endif /* __BSP_KEY_H */

