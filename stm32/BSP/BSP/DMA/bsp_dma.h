/**
 * *****************************************************************************
 * @file        bsp_dma.h
 * @brief       dma��������ͷ�ļ�
 * @author      
 * @date        2024-12-04
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:    stm32f103
 * 
 * *****************************************************************************
 */

#ifndef __BSP_DMA_H 
#define __BSP_DMA_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*DMA��غ궨��*/
#define         DMA_ClockFun                RCC_AHBPeriphClockCmd
#define         DMA_CLK                     RCC_AHBPeriph_DMA1
#define         PeripheralBaseAddr          ADC1_BASE+0X4C
#define         DMA_Channel                 DMA1_Channel1
/*----------------------------------function----------------------------------*/
void DMAInit(void);
/*------------------------------------test------------------------------------*/
#endif /* __BSP_DMA_H */