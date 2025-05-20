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
#include "ringbuff.h"
#include "gps.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t gps_rx_buff;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  �ж����ȼ�����
 * @param  ��
 * @retval ��
 */
static void DMA_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*���ȼ���*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); /*��������Ŀʹ��һ�μ���*/
    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief
 * @note
 * @param  ��
 * @retval ��
 */
static void DMA_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    #if GPS_UBX_IDLE_INTERRUPT_ENABLE
    DMA_InitStructure.DMA_BufferSize = UBX_DMA_BUFF_SIZE;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ubx_active_buff;
    #else
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&gps_rx_buff;
    #endif
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; /*δʹ��FIFO�ⲽ��������*/
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; /*��������ͻ������ģʽ��ͻ������ɵ������ߴ���������*/
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

/*��������ģʽ*/
/*1.*/
/*usart��������ж�+dma���ֽ�ת��*/
/*ʵʱ��ǿ���ɱ߽��ձ߽������ݣ����ж�Ƶ������*/
/*2.*/
/*usart�����ж�+dma���ֽ�ת��*/
/*������һ֡���ݺ��ٽ������ݣ��жϽ���*/

void dma_init(void)
{
    DMA_NVIC_Configure();
    DMA_Configure();
    #if GPS_UBX_IDLE_INTERRUPT_ENABLE
    DMA_DoubleBufferModeCmd(DMA1_Stream2, ENABLE);
    #endif
    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	// DMA_Cmd(DMA1_Stream2, ENABLE);
}

void DMA1_Stream2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
        rb_push(gps_rx_buff, &gnss_rb_mng);
    }
}
