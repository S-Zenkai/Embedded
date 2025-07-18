/**
 ******************************************************************************
 * @file    bsp_usart.c
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
#include "bsp_usart.h"
#include "bsp_systick.h"
#include "pro_common.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*������ɱ�־*/
__IO FlagStatus USART_Send_Done_Flag = SET;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void USART3_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*���ȼ���*/

    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART3-GPIO��������
 * @note
 * @param  ��
 * @retval ��
 */
static void USART3_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#if SBUS_Enable
    /*��������*/
    /*RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
}

/**
 * @brief  USART3����
 * @note
 * @param  ��
 * @retval ��
 */
static void USART3_Configure(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#if SBUS_Enable
    USART_InitStructure.USART_BaudRate = 100000; /*SBUS�涨������*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    ;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b; /*8λ����λ+żУ��*/
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART3, ENABLE);
#endif
}

/******************************************USART2****************************************************/

/**
 * @brief  USART2����(˫��ͨ��)
 * @note
 * @param  ��
 * @retval ��
 */
static void USART_ICC_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    ICC_USART_GPIO_CLK_FUN(ICC_USART_GPIO_CLK, ENABLE);
    ICC_USART_CLK_FUN(ICC_USART_CLK, ENABLE); /*USART2��GPIOA*/
    /*��������*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = ICC_USART_TX_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ICC_USART_TX_PORT, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = ICC_USART_RX_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ICC_USART_RX_PORT, &GPIO_InitStructure);
    /*USART����*/
    USART_InitStructure.USART_BaudRate = 115200;                                    /*������*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*Ӳ��������*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*���������ģʽ*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*��żУ��ģʽ*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*ֹͣλ*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*�����ֳ�*/

    USART_Init(ICC_USART, &USART_InitStructure);
    USART_DMACmd(ICC_USART, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    USART_Cmd(ICC_USART, ENABLE);
}

/****************************************************************************************************/

/**
 * @brief  USART��ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void usart_init(void)
{
    USART3_GPIO_Configure();
    USART3_Configure();
    USART3_NVIC_Configure();

    USART_ICC_Configure();
}

uint8_t counter;

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) == SET)
    {
        /*������б�־*/
        USART3->SR;
        USART3->DR;
        /*���ʣ��δת��ͨ����*/
        //        counter = DMA_GetCurrDataCounter(DMA1_Channel3);
        if (sbus_buff[0] == 0x0F && sbus_buff[24] == 0x00)
        {
            sbus_DF_TC = SET;
        }
        /*�л�������*/
        //        active_buff ^= 1;
        //		active_buff=0;
        //        DMA_Cmd(DMA1_Channel3, DISABLE);
        //        DMA1_Channel3->CMAR = (uint32_t)sbus_buff;
        //        DMA_SetCurrDataCounter(DMA1_Channel3, SBUS_INPUT_CHANNELS);
        //        DMA_Cmd(DMA1_Channel3, ENABLE);
    }
}

/****************************************************���ܺ���*******************************************************/
/**
 * @brief  ���Ͷ���ֽڣ�֧�ֳ�ʱ���
 * @note   ��ʱʱ�����ԭ��usart���͵��ֽ�ʱ�䣺10byte/baubrate��8�ֽ�+1У��λ+1ֹͣλ��������ʱ��Ҫ����һ��������
 * @param  USARTx: ���ھ��
 * @param  Data: ����ָ��
 * @param  length: ���ݳ���
 * @param  timeout: ��ʱʱ��
 * @retval ���ͳɹ����ֽ���
 */
bool USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *Data, uint16_t length, uint32_t timeout)
{
    uint16_t i;
    uint32_t start_time = GetTick();
    if (Data == NULL || length == 0)
    {
        return false;
    }
    for (i = 0; i < length; i++)
    {
        USART_SendData(USARTx, Data[i]);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
        {
            if (GetTick() - start_time > timeout)
            {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief  ʹ��DMA����ָ����������
 * @note
 * @param  DMAy_Channelx: DMAͨ��
 * @param  buffer: ����ָ��
 * @param  len: ���ݳ���
 * @retval ��
 */

void USART_DMA_Send(DMA_Channel_TypeDef *DMAy_Channelx, uint8_t *buffer, uint16_t len)
{
    // ȷ��DMA�Ѿ��رգ�������������
    DMA_Cmd(DMAy_Channelx, DISABLE);
    // �����ڴ��ַ
    DMAy_Channelx->CMAR = (uint32_t)buffer;

    // ����Ҫ�����������
    DMA_SetCurrDataCounter(DMAy_Channelx, len);

    // ʹ��DMA Stream����ʼ����
    DMA_Cmd(DMAy_Channelx, ENABLE);
    
}
