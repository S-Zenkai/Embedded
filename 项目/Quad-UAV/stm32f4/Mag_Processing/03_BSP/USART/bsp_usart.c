/**
 ******************************************************************************
 * @file    bsp_usart.c
 * @author  kai
 * @version V1
 * @date    2025/04/09
 * @brief   usart驱动程序
 ******************************************************************************
 * @attention
 *
 * 注意对F4系列单片机的引脚复用功能，还需要调用对应复用函数
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "bsp_systick.h"
// #include "globle.h"
#include "stdbool.h"
#include "bsp_dma.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  中断优先级配置
 * @param  无
 * @retval 无
 */
// static void USART_NVIC_Configure(void)
//{
//     NVIC_InitTypeDef NVIC_InitStructure;
//     /*优先级组*/
//     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); /*在整个项目使用一次即可*/
//     /*USART1*/
//     NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//     NVIC_Init(&NVIC_InitStructure);
// }

/**************************************USART7配置*********************************************/
/**
 * @brief  USART7_GPIO配置
 * @note
 * @param  无
 * @retval 无
 */
static void USART_DEBUG_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DEBUG_USART_GPIO_CLK_FUN(DEBUG_USART_GPIO_CLK, ENABLE);
    DEBUG_USART_CLK_FUN(DEBUG_USART_CLK, ENABLE); /*USART7、GPIOA*/
    GPIO_PinAFConfig(DEBUG_USART_TX_PORT, DEBUG_USART_AF_PinSource_TX, DEBUG_USART_AF);
    GPIO_PinAFConfig(DEBUG_USART_RX_PORT, DEBUG_USART_AF_PinSource_RX, DEBUG_USART_AF);
    /*引脚配置*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_Pin;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(DEBUG_USART_TX_PORT, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_Pin;
    GPIO_Init(DEBUG_USART_RX_PORT, &GPIO_InitStructure);

    /*USART配置*/
    USART_InitStructure.USART_BaudRate = 115200;                                    /*波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*发送字长*/
    USART_Init(DEBUGE_USART, &USART_InitStructure);
    USART_Cmd(DEBUGE_USART, ENABLE);
}

/**************************************UART4配置*********************************************/

/**
 * @brief  TX-PA0    RX-PA1
 * @note
 * @param  无
 * @retval 无
 */
static void UART4_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /*引脚配置*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
}

static void UART4_Configure(uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    USART_InitStructure.USART_BaudRate = baudrate;                                  /*m8n默认波特率为9600，后续还会轮询调整波特率以适配m8n的真实波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*发送字长*/
    USART_Init(UART4, &USART_InitStructure);
    // USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);/*初始不使能DMA转移数据，等到匹配到正确波特率后再使能*/
}

/**
 * @brief  设置USART4波特率
 * @note
 * @param  baudrate:波特率
 * @retval 无
 */
void USART4_SetBaudRate(uint32_t baud)
{
    /*重置USART与DMA*/
    USART_DeInit(UART4);
    DMA_DeInit(DMA1_Stream2);

    /*配置GPIO、USART、DMA*/
    UART4_GPIO_Configure();
    UART4_Configure(baud);

    /*使能USART、DMA、DMA中断*/
    USART_Cmd(UART4, ENABLE);
    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Stream2, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
}

/**************************************USART7配置*********************************************/
/**
 * @brief  USART7_GPIO配置
 * @note
 * @param  无
 * @retval 无
 */
static void USART_MAVLINK_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    MAVLINK_USART_GPIO_CLK_FUN(MAVLINK_USART_GPIO_CLK, ENABLE);
    MAVLINK_USART_CLK_FUN(MAVLINK_USART_CLK, ENABLE); /*USART7、GPIOA*/
    GPIO_PinAFConfig(MAVLINK_USART_TX_PORT, MAVLINK_USART_AF_PinSource_TX, MAVLINK_USART_AF);
    GPIO_PinAFConfig(MAVLINK_USART_RX_PORT, MAVLINK_USART_AF_PinSource_RX, MAVLINK_USART_AF);
    /*引脚配置*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = MAVLINK_USART_TX_Pin;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(MAVLINK_USART_TX_PORT, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Pin = MAVLINK_USART_RX_Pin;
    GPIO_Init(MAVLINK_USART_RX_PORT, &GPIO_InitStructure);

    /*USART配置*/
    USART_InitStructure.USART_BaudRate = 115200;                                    /*波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*发送字长*/

    USART_Init(MAVLINK_USART, &USART_InitStructure);
    USART_DMACmd(MAVLINK_USART, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(MAVLINK_USART, ENABLE);
}

/**************************************USART6配置*********************************************/
/**
 * @brief  USART6配置，用于双机通信
 * @note
 * @param  无
 * @retval 无
 */
static void USART_ICC_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    ICC_USART_GPIO_CLK_FUN(ICC_USART_GPIO_CLK, ENABLE);
    ICC_USART_CLK_FUN(ICC_USART_CLK, ENABLE); /*USART7、GPIOA*/
    GPIO_PinAFConfig(ICC_USART_TX_PORT, ICC_USART_AF_PinSource_TX, ICC_USART_AF);
    GPIO_PinAFConfig(ICC_USART_RX_PORT, ICC_USART_AF_PinSource_RX, ICC_USART_AF);
    /*引脚配置*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = ICC_USART_TX_Pin;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(ICC_USART_TX_PORT, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Pin = ICC_USART_RX_Pin;
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
/********************************************************************************************/

/**
 * @brief       USART初始化函数
 *
 */
void usart_init(void)
{
    USART_DEBUG_Configure();
    USART_MAVLINK_Configure();
    USART_ICC_Configure();

    UART4_GPIO_Configure();
    UART4_Configure(9600);
#if GPS_UBX_IDLE_INTERRUPT_ENABLE
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
#endif
}

void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_FLAG_IDLE) == SET)
    {

        USART_ClearITPendingBit(UART4, USART_FLAG_IDLE);
    }
}

/**********************************************功能函数********************************************/
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
 * @note   需要在引用函数中添加等待发送完成标志，防止数据丢失
 *         还需将数据放入全局变量中，防止发送未完成，局部变量被释放，发送数据错误
 * @param  DMAy_Channelx: DMA通道
 * @param  buffer: 数据指针
 * @param  len: 数据长度
 * @retval 无
 */
void USART_DMA_Send(DMA_Stream_TypeDef *DMAy_Streamx, uint8_t *buffer, uint16_t len)
{
    /*下面注释的程序需要在外部调用该函数时添加，即检查发送完成标志以及将要发送的数据放入全局变量中*/
    /*全局变量是为了防止发送未完成，局部变量被释放，发送数据错误*/
    // while (USART_Send_Done_Flag == RESET)
    // {
    // };
    // USART_Send_Done_Flag = RESET;
    //memcpy(ICC_TX_Buff, buffer, len);

    // 确保DMA已经关闭，才能重新配置
    DMA_Cmd(DMAy_Streamx, DISABLE);
    // 等待DMA可以被配置
    while (DMA_GetCmdStatus(DMAy_Streamx) != DISABLE)
        ;
    // 设置内存地址
    DMAy_Streamx->M0AR = (uint32_t)buffer;

    // 设置要传输的数据量
    DMA_SetCurrDataCounter(DMAy_Streamx, len);

    // 使能DMA Stream，开始传输
    DMA_Cmd(DMAy_Streamx, ENABLE);
}

/**
 * @brief  接收多个字节，支持超时检测
 * @note   超时时间设计原则：需要根据通信模块设置（可根据参考手册模块响应时间设置）
 * @param  USARTx: 串口句柄
 * @param  Data: 数据指针
 * @param  length: 数据长度
 * @param  timeout: 超时时间
 * @retval 接收成功的字节数
 */
bool USART_ReceiveBytes(USART_TypeDef *USARTx, uint8_t *Data, uint32_t length, uint32_t timeout)
{
    uint32_t i;
    uint32_t start_time = GetTick();
    for (i = 0; i < length; i++)
    {
        Data[i] = USART_ReceiveData(USARTx);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
        {
            if (GetTick() - start_time > timeout)
            {
                return false;
            }
        }
    }
    return true;
}

// /**
//  * @brief
//  *
//  * @param USARTx -
//  * @param Data -
//  */
// void USART_SendByte(USART_TypeDef *USARTx, uint16_t Data)
// {
//     USART_SendData(USARTx, Data);
//     while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
//         ;
// }

// uint8_t USART_ReceiveByte(USART_TypeDef *USARTx)
// {
//     uint8_t data;
//     data = USART_ReceiveData(USARTx);
//     while (USART_GetFlagStatus(UART7, USART_FLAG_RXNE) == RESET)
//         ;
//     return data;
// }

// void USART_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch)
// {
//     uint8_t temp_h, temp_l;

//     /* 取出高八位 */
//     temp_h = (ch & 0XFF00) >> 8;
//     /* 取出低八位 */
//     temp_l = ch & 0XFF;

//     /* 发送高八位 */
//     USART_SendData(pUSARTx, temp_h);
//     while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
//         ;

//     /* 发送低八位 */
//     USART_SendData(pUSARTx, temp_l);
//     while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
//         ;
// }

// 重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
    /* 发送一个字节数据到串口 */
    USART_SendData(UART7, (uint8_t)ch);

    /* 等待发送完毕 */
    while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET)
        ;

    return (ch);
}

/// 重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
    /* 等待串口输入数据 */
    while (USART_GetFlagStatus(UART7, USART_FLAG_RXNE) == RESET)
        ;

    return (int)USART_ReceiveData(UART7);
}

/*------------------------------------test------------------------------------*/
