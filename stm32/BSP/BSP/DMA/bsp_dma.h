/**
 * *****************************************************************************
 * @file        bsp_dma.h
 * @brief       dma驱动程序头文件
 * @author      
 * @date        2024-12-04
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:    stm32f103
 * 
 * *****************************************************************************
 */

#ifndef __BSP_DMA_H 
#define __BSP_DMA_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*DMA相关宏定义*/
#define         DMA_ClockFun                RCC_AHBPeriphClockCmd
#define         DMA_CLK                     RCC_AHBPeriph_DMA1
#define         PeripheralBaseAddr          ADC1_BASE+0X4C
#define         DMA_Channel                 DMA1_Channel1
/*----------------------------------function----------------------------------*/
void DMAInit(void);
/*------------------------------------test------------------------------------*/
#endif /* __BSP_DMA_H */
