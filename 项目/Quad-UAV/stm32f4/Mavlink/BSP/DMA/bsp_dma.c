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
#include "ringbuff.h"
#include "gps.h"
#include "ProConfig.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t gps_rx_temp;
uint8_t mavlink_rx_temp;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/********************************DMA1_Stream2_Channel_4(UART4_RX)**************************************/
/**
 * @brief  中断优先级配置
 * @param  无
 * @retval 无
 */
static void DMA_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA1_S2_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMA1_S2_IRQ_SUB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  DMA1_Stream2_Channel_4_Configure
 * @note
 * @param  无
 * @retval 无
 */
void DMA1_S2_C4_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    #if GPS_UBX_IDLE_INTERRUPT_ENABLE
    DMA_InitStructure.DMA_BufferSize = UBX_DMA_BUFF_SIZE;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ubx_active_buff;
    #else
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&gps_rx_temp;
    #endif
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; /*未使能FIFO这步配置无用*/
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; /*配置数据突发传输模式，突发传输可单次总线传输多个数据*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream2, &DMA_InitStructure);
    #if GPS_UBX_IDLE_INTERRUPT_ENABLE
    DMA_DoubleBufferModeConfig(DMA1_Stream2, (uint32_t)ubx_dma_buff[1], DMA_Memory_0);
    #endif
}
/********************************DMA1_Stream6_Channel_5(UART8_RX)**************************************/
void DMA1_S6_C5_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /*中断优先级配置*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA1_S6_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMA1_S6_IRQ_SUB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    /*DMA配置*/
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&mavlink_rx_temp;
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; /*未使能FIFO这步配置无用*/
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; /*配置数据突发传输模式，突发传输可单次总线传输多个数据*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART8->DR;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Stream6, ENABLE);
}




void dma_init(void)
{
    DMA_NVIC_Configure();
    DMA1_S2_C4_Configure();
    #if GPS_UBX_IDLE_INTERRUPT_ENABLE
    DMA_DoubleBufferModeCmd(DMA1_Stream2, ENABLE);
    #endif

    DMA1_S6_C5_Configure();
}





