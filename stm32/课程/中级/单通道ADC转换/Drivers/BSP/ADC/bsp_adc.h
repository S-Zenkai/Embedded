/**
 * *****************************************************************************
 * @file        bsp_adc.h
 * @brief       adc驱动程序头文件
 * @author      
 * @date        2024-11-28
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:    stm32f103
 * 
 * *****************************************************************************
 */

#ifndef __BSP_ADC_H 
#define __BSP_ADC_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*ADC相关GPIO宏定义*/
#define         ADC_GPIO_ClockFun           RCC_APB2PeriphClockCmd
#define         ADC_GPIO_CLK                RCC_APB2Periph_GPIOC
#define         ADC_GPIO_Port               GPIOC
#define         ADC_GPIO_Pin                GPIO_Pin_1
/*ADC相关宏定义*/
#define         ADC_ClockFun                RCC_APB2PeriphClockCmd
#define         ADC_CLK                RCC_APB2Periph_ADC1
#define         ADC                         ADC1

/*----------------------------------function----------------------------------*/
void ADCInit(void);
uint16_t GetConvValue(void);
/*------------------------------------test------------------------------------*/
#endif /* __BSP_ENCODER_H */
