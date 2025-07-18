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
#include "pro_common.h"
#include "ringbuff.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t icc_rx_temp;
uint8_t icc_tx_temp;/*��������ݣ��������޸Ļ����������ͳ���*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//static void DMA_NVIC_Configure(void)
//{
//    NVIC_InitTypeDef NVIC_InitStructure;
//    /*���ȼ���*/
//    /*USART1*/
//    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;/*�������ȼ��趨��Щ������Ϊ�����ж���ռ����ɵ��ӳ�*/
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_Init(&NVIC_InitStructure);
//}

/**
  * @brief  
  * @note   
  * @param  ��
  * @retval ��
  */
static void DMA_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitStructure.DMA_BufferSize = SBUS_INPUT_CHANNELS;/*sbus����֡�ֽ���*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&sbus_buff[0];
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;/*������Ҫȡ��ַ��*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    // DMA_ITConfig(DMA1_Channel3, DMA_IT_TC ,ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}

/**
  * @brief  DMA����(˫��ͨ��)
  * @note   
  * @param  ��
  * @retval ��
  */
static void DMA1_ICC_Configure(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /*�ж����ȼ�����*/
    /*DMA_RX*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA1_C6_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMA1_C6_IRQ_SUB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);
    /*DMA_TX*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DMA1_C7_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = DMA1_C7_IRQ_SUB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    /*����DMA1_Channel6-RX*/
    DMA_InitStructure.DMA_BufferSize = 1;/*����֡�ֽ���*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&icc_rx_temp;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;/*������Ҫȡ��ַ��*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC ,ENABLE);
    DMA_Cmd(DMA1_Channel6, ENABLE);

    /*����DMA1_Channel7-TX*/
    DMA_InitStructure.DMA_BufferSize = 1;/*�������޸�*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&icc_tx_temp;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC ,ENABLE);
    /*���ﲻʹ��DMA���ͣ��ڷ��ͺ�����ʹ��*/
    
}



extern ringbuff_t ICC_RX_RingBuffMgr;
void DMA1_Channel6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC6) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC6);
        rb_push(icc_rx_temp, &ICC_RX_RingBuffMgr);
    }
}

extern __IO FlagStatus ICC_Send_Done;
/**
  * @brief  ����ת������ж�(ת�Ƶ�usart-dr)
  * @note   �����ж�ֻ��˵������ת����ɣ�����˵�������Ѿ�������ɣ���Ҫ��ȷ�ж����ݷ������
  *        ��Ҫ����USART_IT_TC�жϣ���DMA1_Channel7_IRQHandlerʹ�ܸ��жϡ�
  *        ����ĿӦ���ǲ���Ҫ��ô��ȷ��
  * @param  ��
  * @retval ��
  */
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC7) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC7);
        ICC_Send_Done = SET;
    }
}

void dma_init(void)
{
    // DMA_NVIC_Configure();
      DMA_Configure();
	  DMA1_ICC_Configure();
}


