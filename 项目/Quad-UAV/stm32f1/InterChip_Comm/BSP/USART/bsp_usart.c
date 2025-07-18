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

/*发送完成标志*/
__IO FlagStatus USART_Send_Done_Flag = SET;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void USART3_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*优先级组*/

    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  USART3-GPIO引脚配置
 * @note
 * @param  无
 * @retval 无
 */
static void USART3_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#if SBUS_Enable
    /*引脚配置*/
    /*RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
}

/**
 * @brief  USART3配置
 * @note
 * @param  无
 * @retval 无
 */
static void USART3_Configure(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#if SBUS_Enable
    USART_InitStructure.USART_BaudRate = 100000; /*SBUS规定波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    ;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b; /*8位数据位+偶校验*/
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART3, ENABLE);
#endif
}

/******************************************USART2****************************************************/

/**
 * @brief  USART2配置(双机通信)
 * @note
 * @param  无
 * @retval 无
 */
static void USART_ICC_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    ICC_USART_GPIO_CLK_FUN(ICC_USART_GPIO_CLK, ENABLE);
    ICC_USART_CLK_FUN(ICC_USART_CLK, ENABLE); /*USART2、GPIOA*/
    /*引脚配置*/
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
    /*USART配置*/
    USART_InitStructure.USART_BaudRate = 115200;                                    /*波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*发送字长*/

    USART_Init(ICC_USART, &USART_InitStructure);
    USART_DMACmd(ICC_USART, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    USART_Cmd(ICC_USART, ENABLE);
}

/****************************************************************************************************/

/**
 * @brief  USART初始化
 * @note
 * @param  无
 * @retval 无
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
        /*清楚空闲标志*/
        USART3->SR;
        USART3->DR;
        /*获得剩余未转移通道数*/
        //        counter = DMA_GetCurrDataCounter(DMA1_Channel3);
        if (sbus_buff[0] == 0x0F && sbus_buff[24] == 0x00)
        {
            sbus_DF_TC = SET;
        }
        /*切换缓冲区*/
        //        active_buff ^= 1;
        //		active_buff=0;
        //        DMA_Cmd(DMA1_Channel3, DISABLE);
        //        DMA1_Channel3->CMAR = (uint32_t)sbus_buff;
        //        DMA_SetCurrDataCounter(DMA1_Channel3, SBUS_INPUT_CHANNELS);
        //        DMA_Cmd(DMA1_Channel3, ENABLE);
    }
}

/****************************************************功能函数*******************************************************/
/**
 * @brief  发送多个字节，支持超时检测
 * @note   超时时间设计原则：usart发送单字节时间：10byte/baubrate（8字节+1校验位+1停止位）；设置时需要加入一定余量，
 * @param  USARTx: 串口句柄
 * @param  Data: 数据指针
 * @param  length: 数据长度
 * @param  timeout: 超时时间
 * @retval 发送成功的字节数
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
 * @brief  使用DMA发送指定长度数据
 * @note
 * @param  DMAy_Channelx: DMA通道
 * @param  buffer: 数据指针
 * @param  len: 数据长度
 * @retval 无
 */

void USART_DMA_Send(DMA_Channel_TypeDef *DMAy_Channelx, uint8_t *buffer, uint16_t len)
{
    // 确保DMA已经关闭，才能重新配置
    DMA_Cmd(DMAy_Channelx, DISABLE);
    // 设置内存地址
    DMAy_Channelx->CMAR = (uint32_t)buffer;

    // 设置要传输的数据量
    DMA_SetCurrDataCounter(DMAy_Channelx, len);

    // 使能DMA Stream，开始传输
    DMA_Cmd(DMAy_Channelx, ENABLE);
    
}
