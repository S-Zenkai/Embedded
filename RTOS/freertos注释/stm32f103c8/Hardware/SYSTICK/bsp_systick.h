#ifndef _BSP_SYSTICK_H
#define _BSP_SYSTICK_H
#include "stm32f10x.h"
#include "bsp_sys.h"

#if SYS_SUPPORT_OS
void Delay_us(__IO uint32_t us);
void Delay_ms(__IO uint32_t ms);
void SisTick_Init(void);
void SystickInit(uint16_t sysclk);

#else
void SysTick_Init(void);
uint32_t GetTick(void);
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);
#endif

#endif /*_BSP_SYSTICK_H*/
