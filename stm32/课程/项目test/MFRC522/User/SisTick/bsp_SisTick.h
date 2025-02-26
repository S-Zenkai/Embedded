#ifndef _BSP_SYSTICK_H
#define _BSP_SYSTICK_H
#include "stm32f10x.h"

void Delay_SisTick_us(__IO uint32_t us);
void Delay_SisTick_ms(__IO uint32_t ms);
void SisTick_Init(void);


#endif /*_BSP_SYSTICK_H*/
