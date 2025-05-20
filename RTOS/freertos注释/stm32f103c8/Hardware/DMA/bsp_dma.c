/**
  ******************************************************************************
  * @file    bsp_dma.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/27
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_dma.h"
#include "globle.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void DMA_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*优先级组*/
    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;/*这里优先级设定高些避免因为其他中断抢占而造成的延迟*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  
  * @note   
  * @param  无
  * @retval 无
  */
static void DMA_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitStructure.DMA_BufferSize = SBUS_INPUT_CHANNELS;/*sbus数据帧字节数*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)sbus_buff[0];
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;/*这里需要取地址吗*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    // DMA_ITConfig(DMA1_Channel3, DMA_IT_TC ,ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}

void dma_init(void)
{
    // DMA_NVIC_Configure();
    DMA_Configure();
}

// void DMA1_Channel3_IRQHandler(void)
// {
//     if(DMA_GetITStatus(DMA1_IT_TC3)==SET)
//     {
//         DMA_ClearITPendingBit(DMA1_IT_TC3);
//         sbus_DF_TC = SET;
//         /*切换接收缓冲区*/
//         decode_buff = &sbus_buff[SBUS_INPUT_CHANNELS];
//     }
// }
