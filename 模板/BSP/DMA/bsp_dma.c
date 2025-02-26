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
 * ʵ��ƽ̨:    stm32f103vet6
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_dma.h"
/*-----------------------------------macro------------------------------------*/
#define BuffSZ 3
/*----------------------------------variable----------------------------------*/
uint16_t DataBuffer[BuffSZ] = {0};/*���ڴ��DMAת������*/
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
static void DMA_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    DMA_ClockFun(DMA_CLK, ENABLE);
    DMA_InitStructure.DMA_BufferSize = BuffSZ;/*���ݻ�������С*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;/*���ݴ��䷽��*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;/*M2Mģʽ*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DataBuffer;/*M��ַ�����ݻ�������ַ��*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;/*���ݻ��������ݴ�С*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;/*M�����ݻ�������ָ���Ƿ����*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;/*DMA����ģʽ��һ�δ����ѭ������*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = PeripheralBaseAddr;/*�����ַ*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;/*�������ݴ�С*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;/*����ָ���Ƿ����*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;/*���ȼ�*/
    DMA_Init(DMA_Channel, &DMA_InitStructure);/*��ʼ����ÿ��DMAͨ����Ӧ��ͬ����DMA������*/
}

void DMAInit(void)
{
    DMA_Configure();
    DMA_Cmd(DMA_Channel, ENABLE);
}
/*------------------------------------test------------------------------------*/
