#ifndef _BSP_SYSTICK_H
#define _BSP_SYSTICK_H
#include "stm32f10x.h"

void Delay_us(__IO uint32_t us);
void Delay_ms(__IO uint32_t ms);
void SisTick_Init(void);

void SystickInit(uint16_t sysclk);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);


#endif /*_BSP_SYSTICK_H*/
