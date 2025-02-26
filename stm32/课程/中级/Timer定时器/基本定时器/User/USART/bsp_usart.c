#include "bsp_usart.h"

/*
static void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel=Debug_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
}
*/

//USART��ʼ��
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//��ʱ��
	Debug_RCC_PeriphClockCmd(Debug_USART_GPIO_CLC|Debug_USART_CLC, ENABLE);
	
	//GPIO�����ʼ��
	GPIO_InitStructure.GPIO_Pin=Debug_USART_GPIO_TXPin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(Debug_USART_GPIO, &GPIO_InitStructure);
	
	//GPIO�����ʼ��
	GPIO_InitStructure.GPIO_Pin=Debug_USART_GPIO_RXPin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(Debug_USART_GPIO, &GPIO_InitStructure);
	
	//USART1��ʼ��
	USART_InitStructure.USART_BaudRate=Debug_USART_BaudRate;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(Debug_USART, &USART_InitStructure);
	
	//ʹ���ж�
//	NVIC_Config();
//	USART_ITConfig(Debug_USART, USART_IT_RXNE, ENABLE);
	
	//ʹ�ܴ���
	USART_Cmd(Debug_USART, ENABLE);
}


//����һ���ֽ�
void SendData_Byte(USART_TypeDef* USARTx, uint8_t Data)
{
	USART_SendData(USARTx,Data);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}

//���������ֽ�
void SendData_TwoBytes(USART_TypeDef* USARTx, uint16_t Data)
{
	uint8_t tmp_h,tmp_l;
	tmp_h=(Data&0xFF00)>>8;
	tmp_l=(Data&0xFF);
	SendData_Byte(USARTx,tmp_h);
	SendData_Byte(USARTx,tmp_l);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);//ΪʲôҪ���TC��־λ��
}

//����һ���ַ���
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

//����һ������
void SendData_Arr(USART_TypeDef* USARTx, uint8_t* pData, uint8_t sz)
{
	uint8_t i=0;
	for(i=0;i<sz;i++)
	{
		SendData_Byte(USARTx,*(pData+i));
	}
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}


//�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(Debug_USART, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(Debug_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(Debug_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(Debug_USART);
}




