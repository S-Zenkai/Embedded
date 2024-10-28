#include "dma_m2p.h"

//对变量的定义应该在c文件中，在h文件中会造成变量从定义的报错
//const uint32_t Asour[sz]={0x01020304,0x05060708,0x090A0B0C,0x0D0E0F10,
//                        0x11121314,0x15161718,0x191A1B1C,0x1D1E1F20,
//                        0x21222324,0x25262728,0x292A2B2C,0x2D2E2F30,
//                        0x31323334,0x35363738,0x393A3B3C,0x3D3E3F40,
//                        0x41424344,0x45464748,0x494A4B4C,0x4D4E4F50,
//                        0x51525354,0x55565758,0x595A5B5C,0x5D5E5F60,
//                        0x61626364,0x65666768,0x696A6B6C,0x6D6E6F70,
//                        0x71727374,0x75767778,0x797A7B7C,0x7D7E7F80};
//uint16_t sz=sizeof(Asour)/sizeof(Asour[1]);

uint8_t Asour[sz];
void DMA_M2P_Config(void)
{
	RCC_AHBPeriphClockCmd(DMA_M2M_CLC, ENABLE);
	DMA_InitTypeDef DMA_M2M_InitStruct;
	DMA_M2M_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)DMA_M2M_Dest;
	DMA_M2M_InitStruct.DMA_MemoryBaseAddr=(uint32_t)DMA_M2M_Sour;
	DMA_M2M_InitStruct.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_M2M_InitStruct.DMA_BufferSize=DMA_M2M_Buff_Sz;
	DMA_M2M_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_M2M_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_M2M_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	DMA_M2M_InitStruct.DMA_MemoryDataSize=DMA_PeripheralDataSize_Byte;
	DMA_M2M_InitStruct.DMA_Mode=DMA_Mode_Normal;
	DMA_M2M_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_M2M_InitStruct.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(DMA_M2M_Channelx, &DMA_M2M_InitStruct);
	DMA_Cmd(DMA_M2M_Channelx, ENABLE);
}

void USART_Config(void)
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
	
	//USART1初始化
	USART_InitStructure.USART_BaudRate=Debug_USART_BaudRate;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx;
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


//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(Debug_USART, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(Debug_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}


