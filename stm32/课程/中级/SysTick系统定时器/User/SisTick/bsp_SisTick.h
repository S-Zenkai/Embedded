#ifndef _BSP_SYSTICK_H
#define _BSP_SYSTICK_H
#include "stm32f10x.h"//Ϊʲô����������λ�õ��������ֺܶ����
//core_cm3.h��ʹ����stm32f10x.h�����IRQn���ʱ����Ȱ���stm32f10x.h�ſ���
void Delay_SisTick_us(uint32_t us);
void Delay_SisTick_ms(__IO uint32_t ms);
void SisTick_Init(void);
void SisTick_Counter_Decrement(void);


#endif /*_BSP_SYSTICK_H*/
