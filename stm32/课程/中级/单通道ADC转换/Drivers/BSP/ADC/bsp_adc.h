/**
 * *****************************************************************************
 * @file        bsp_adc.h
 * @brief       adc��������ͷ�ļ�
 * @author      
 * @date        2024-11-28
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:    stm32f103
 * 
 * *****************************************************************************
 */

#ifndef __BSP_ADC_H 
#define __BSP_ADC_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*ADC���GPIO�궨��*/
#define         ADC_GPIO_ClockFun           RCC_APB2PeriphClockCmd
#define         ADC_GPIO_CLK                RCC_APB2Periph_GPIOC
#define         ADC_GPIO_Port               GPIOC
#define         ADC_GPIO_Pin                GPIO_Pin_1
/*ADC��غ궨��*/
#define         ADC_ClockFun                RCC_APB2PeriphClockCmd
#define         ADC_CLK                RCC_APB2Periph_ADC1
#define         ADC                         ADC1

/*----------------------------------function----------------------------------*/
void ADCInit(void);
uint16_t GetConvValue(void);
/*------------------------------------test------------------------------------*/
#endif /* __BSP_ENCODER_H */
