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
#include "globle.h"
#include "stdbool.h"
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
	static void USART_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*优先级组*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); /*在整个项目使用一次即可*/
    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**************************************USART7配置*********************************************/
/**
 * @brief  USART7_GPIO配置
 * @note
 * @param  无
 * @retval 无
 */
static void USART7_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);
    /*引脚配置*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/**
 * @brief  USART7_配置
 * @note
 * @param  无
 * @retval 无
 */
static void USART7_Configure(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);                           /*USART7、GPIOA*/
    USART_InitStructure.USART_BaudRate = 115200;                                    /*波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*发送字长*/
    USART_Init(UART7, &USART_InitStructure);
    USART_Cmd(UART7, ENABLE);
}
/********************************************************************************************/
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

static void UART4_Configure(void)
{
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600;                                      /*m8n默认波特率为9600，后续还会轮询调整波特率以适配m8n的真实波特率*/
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
void USART4_SetBaudRate( uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*发送字长*/
    USART_Init(UART4, &USART_InitStructure);
}

/********************************************************************************************/

/**
 * @brief       USART初始化函数
 *
 */
void usart_init(void)
{
    USART7_GPIO_Configure();
    USART7_Configure();

    UART4_GPIO_Configure();
    UART4_Configure();
    #if GPS_UBX_IDLE_INTERRUPT_ENABLE
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
    #endif
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    // USART_Cmd(UART4, ENABLE);
}


void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_FLAG_IDLE) == SET)
    {
        
        USART_ClearITPendingBit(UART4, USART_FLAG_IDLE);
    }
}


/**
  * @brief  发送多个字节，支持超时检测
  * @note   超时时间设计原则：usart发送单字节时间：10byte/baubrate（8字节+1校验位+1停止位）；设置时需要加入一定余量，
  * @param  USARTx: 串口句柄
  * @param  Data: 数据指针
  * @param  length: 数据长度
  * @param  timeout: 超时时间
  * @retval 发送成功的字节数
  */
bool USART_SendBytes(USART_TypeDef *USARTx, uint8_t *Data, uint32_t length, uint32_t timeout)
{
    uint32_t i;
    uint32_t start_time = GetTick();
    if(Data==NULL||length==0)
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
