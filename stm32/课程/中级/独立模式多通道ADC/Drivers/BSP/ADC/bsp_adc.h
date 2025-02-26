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
#define         ADCx_GPIO_ClockFun           RCC_APB2PeriphClockCmd
#define         ADCx_GPIO_CLK                RCC_APB2Periph_GPIOC
#define         ADCx_IN10_Port               GPIOC
#define         ADCx_IN10_Pin                GPIO_Pin_0
#define         ADCx_IN11_Port               GPIOC
#define         ADCx_IN11_Pin                GPIO_Pin_1
#define         ADCx_IN12_Port               GPIOC
#define         ADCx_IN12_Pin                GPIO_Pin_2
/*adc转换通道数*/
#define         NbrOfChannel                 3
/*ADC相关宏定义*/
#define         ADCx_ClockFun                RCC_APB2PeriphClockCmd
#define         ADCx_CLK                     RCC_APB2Periph_ADC1
#define         ADCx                         ADC1


/*----------------------------------function----------------------------------*/
void ADCInit(void);
uint16_t GetConvValue(void);
/*------------------------------------test------------------------------------*/
#endif /* __BSP_ENCODER_H */
