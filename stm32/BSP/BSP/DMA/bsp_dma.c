/**
 * *****************************************************************************
 * @file        bsp_dma.c
 * @brief       
 * @author      
 * @date        2024-12.04
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:    stm32f103vet6
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_dma.h"
/*-----------------------------------macro------------------------------------*/
#define BuffSZ 3
/*----------------------------------variable----------------------------------*/
uint16_t DataBuffer[BuffSZ] = {0};/*用于存放DMA转运数据*/
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
static void DMA_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    DMA_ClockFun(DMA_CLK, ENABLE);
    DMA_InitStructure.DMA_BufferSize = BuffSZ;/*数据缓冲区大小*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;/*数据传输方向*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;/*M2M模式*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DataBuffer;/*M基址（数据缓冲区地址）*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;/*数据缓冲区数据大小*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;/*M（数据缓冲器）指针是否递增*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;/*DMA传输模式，一次传输或循环传输*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = PeripheralBaseAddr;/*外设基址*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;/*外设数据大小*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;/*外设指针是否递增*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;/*优先级*/
    DMA_Init(DMA_Channel, &DMA_InitStructure);/*初始化，每个DMA通道对应不同外设DMA请求功能*/
}

void DMAInit(void)
{
    DMA_Configure();
    DMA_Cmd(DMA_Channel, ENABLE);
}
/*------------------------------------test------------------------------------*/
