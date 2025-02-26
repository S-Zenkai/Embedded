/**
 * *****************************************************************************
 * @file        bsp_usart.c
 * @brief       
 * @author      
 * @date        2024-12-26
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_usart.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
static void USART_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*优先级组*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);/*在整个项目使用一次即可*/
    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    /*USART2*/
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}
/**
 * @brief       USART1配置函数
 * 
 */
static void USART1_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    /*时钟*/
    USART1_CLK_FUN(USART1_CLK, ENABLE);/*USART1、GPIOA*/
    /*中断配置*/
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /*引脚配置*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = USART1_GPIOPin_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART1_GPIO_TX, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = USART1_GPIOPin_RX;
    GPIO_Init(USART1_GPIO_RX, &GPIO_InitStructure);
    /*USART配置*/
    USART_InitStructure.USART_BaudRate = 115200;/*波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;/*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;/*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;/*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*发送字长*/
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}


/**
 * @brief       USART2配置函数
 * 
 */
static void USART2_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    /*时钟*/
    USART2_CLK_FUN(USART2_CLK, ENABLE);/*USART2、GPIOA*/
    /*中断配置*/
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    /*引脚配置*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = USART2_GPIOPin_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART2_GPIO_TX, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = USART2_GPIOPin_RX;
    GPIO_Init(USART2_GPIO_RX, &GPIO_InitStructure);
    /*USART配置*/
    USART_InitStructure.USART_BaudRate = 115200;/*波特率*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*硬件流控制*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;/*发送与接收模式*/
    USART_InitStructure.USART_Parity = USART_Parity_No;/*奇偶校验模式*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;/*停止位*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*发送字长*/
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
}

/**
 * @brief       USART初始化函数
 * 
 */
void USARTInit(void)
{
    USART_NVIC_Configure();
    USART1_Configure();
    USART2_Configure();
}

void USARTx_SendByte(USART_TypeDef* USARTx, uint16_t Data)
{
    USART_SendData(USART1, Data);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET)
        ;
}

uint8_t USARTx_ReceiveByte(USART_TypeDef* USARTx)
{
    uint8_t data;
    data = USART_ReceiveData(USARTx);
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)==RESET)
        ;
    return data;
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_TXE)==SET)
    {
        USART_ClearFlag(USART1, USART_IT_TXE);
    }
    if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)
    {
        USART_ClearFlag(USART1, USART_IT_RXNE);
    }
}

void USART2_IRQHandler(void)
{
    uint8_t data;
    if(USART_GetITStatus(USART2, USART_IT_TXE)==SET)
    {
        USART_ClearFlag(USART2, USART_IT_TXE);
    }
    if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)
    {
        data = USARTx_ReceiveByte(USART2);
        USARTx_SendByte(USART2, data);
        USART_ClearFlag(USART2, USART_IT_RXNE);
    }
}

//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
/*------------------------------------test------------------------------------*/
#if 0
//static void NVIC_Config(void)
//{
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	NVIC_InitStructure.NVIC_IRQChannel=Debug_USART_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
//	NVIC_Init(&NVIC_InitStructure);
//}

//USART初始化
static void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//开时钟
	Debug_RCC_PeriphClockCmd(Debug_USART_GPIO_CLC|Debug_USART_CLC, ENABLE);
	
	//GPIO输出初始化
	GPIO_InitStructure.GPIO_Pin=Debug_USART_GPIO_TXPin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(Debug_USART_GPIO, &GPIO_InitStructure);
	
	//GPIO输入初始化
	GPIO_InitStructure.GPIO_Pin=Debug_USART_GPIO_RXPin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(Debug_USART_GPIO, &GPIO_InitStructure);
	
	//USART1初始化
	USART_InitStructure.USART_BaudRate=Debug_USART_BaudRate;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(Debug_USART, &USART_InitStructure);
	
	//使能中断
//	NVIC_Config();
//	USART_ITConfig(Debug_USART, USART_IT_RXNE, ENABLE);
	
	//使能串口
	USART_Cmd(Debug_USART, ENABLE);
}

void UsartInit(void)
{
    USART_Config();
}

//发送一个字节
void SendData_Byte(USART_TypeDef* USARTx, uint8_t Data)
{
	USART_SendData(USARTx,Data);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}

//发送两个字节
void SendData_TwoBytes(USART_TypeDef* USARTx, uint16_t Data)
{
	uint8_t tmp_h,tmp_l;
	tmp_h=(Data&0xFF00)>>8;
	tmp_l=(Data&0xFF);
	SendData_Byte(USARTx,tmp_h);
	SendData_Byte(USARTx,tmp_l);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);//为什么要检查TC标志位？
}

//发送一个字符串
void SendData_Char(USART_TypeDef* USARTx, char* pData)
{
	uint8_t i=0;
	do
	{
		SendData_Byte(USARTx,*(pData+i));
		i++;
	}while(*(pData+i)!='\0');
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

//发送一个数组
void SendData_Arr(USART_TypeDef* USARTx, uint8_t* pData, uint8_t sz)
{
	uint8_t i=0;
	for(i=0;i<sz;i++)
	{
		SendData_Byte(USARTx,*(pData+i));
	}
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}


//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(Debug_USART, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(Debug_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(Debug_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(Debug_USART);
}
#endif



