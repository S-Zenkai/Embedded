#ifndef _BSP_SYSTICK_H
#define _BSP_SYSTICK_H
#include "stm32f10x.h"//为什么两个包含的位置调换后会出现很多错误？
//core_cm3.h中使用了stm32f10x.h定义的IRQn，故必须先包含stm32f10x.h才可以
#include "core_cm3.h"
void Delay_SisTick_us(uint32_t us);



#endif /*_BSP_SYSTICK_H*/
