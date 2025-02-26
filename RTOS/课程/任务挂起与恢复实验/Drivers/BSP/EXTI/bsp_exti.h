
#ifndef __BSP_EXTI_H 
#define __BSP_EXTI_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*EXTI���������GPIO��*/
#define             EXTI_GPIO_APBxCLK_FUN           RCC_APB2PeriphClockCmd
#define             EXTI_GPIO_CLK                   RCC_APB2Periph_GPIOC
#define             EXTI_GPIO_Port                  GPIOC
#define             EXTI_GPIO_Pin                   GPIO_Pin_13
/*AFIO��غ꣬��stm32�У�AFIO������GPIO������ӳ���Լ�EXTI�ж�Դѡ��*/
#define             EXTI_AFIO_APBxCLK_FUN           RCC_APB2PeriphClockCmd
#define             EXTI_AFIO_CLK                   RCC_APB2Periph_AFIO
#define             EXTI_AFIO_PortSource            GPIO_PortSourceGPIOC
#define             EXTI_AFIO_PinSource             GPIO_PinSource13
/*EXTI��غ�*/
#define             EXTI_Linex                      EXTI_Line13
#define             Exti_CLK                        RCC_APB2Periph_AFIO
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void ExtiInit(void);
/*------------------------------------test------------------------------------*/


#endif	/* __BSP_EXTI_H */
