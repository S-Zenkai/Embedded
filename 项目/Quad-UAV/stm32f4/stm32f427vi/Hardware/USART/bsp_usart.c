/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  kai
  * @version V1
  * @date    2025/2/19
  * @brief   usart��������
  ******************************************************************************
  * @attention
  *
  * ע���F4ϵ�е�Ƭ�������Ÿ��ù��ܣ�����Ҫ���ö�Ӧ���ú���
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  �ж����ȼ�����
  * @param  ��
  * @retval ��
  */
static void USART_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*���ȼ���*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);/*��������Ŀʹ��һ�μ���*/
    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  
  * @note   
  *         
  * @param  ��
  * @retval ��
  */
static void USART7_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    /*ʱ��*/
    USART7_CLK_FUN(USART7_CLK, ENABLE);/*USART7��GPIOA*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    /*�ж�����*/

    /*��������*/
    /*TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = USART7_GPIOPin_TX;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(USART7_GPIO_TX, &GPIO_InitStructure);
    /*RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = USART7_GPIOPin_RX;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(USART7_GPIO_RX, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7);
    /*USART����*/
    USART_InitStructure.USART_BaudRate = 115200;/*������*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*Ӳ��������*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;/*���������ģʽ*/
    USART_InitStructure.USART_Parity = USART_Parity_No;/*��żУ��ģʽ*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;/*ֹͣλ*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*�����ֳ�*/
    USART_Init(UART7, &USART_InitStructure);
    USART_Cmd(UART7, ENABLE);
}

/**
 * @brief       USART��ʼ������
 * 
 */
void USARTInit(void)
{
    USART7_Configure();
}

/**
 * @brief 
 * 
 * @param USARTx - 
 * @param Data - 
 */
 void USARTx_SendByte(USART_TypeDef* USARTx, uint16_t Data)
 {
     USART_SendData(USARTx, Data);
     while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE)==RESET)
         ;
 }


uint8_t USARTx_ReceiveByte(USART_TypeDef* USARTx)
{
    uint8_t data;
    data = USART_ReceiveData(USARTx);
    while(USART_GetFlagStatus(UART7, USART_FLAG_RXNE)==RESET)
        ;
    return data;
}


void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}


void USART7_IRQHandler(void)
{
    if(USART_GetITStatus(UART7, USART_IT_TXE)==SET)
    {
        USART_ClearFlag(UART7, USART_IT_TXE);
    }
    if(USART_GetITStatus(UART7, USART_IT_RXNE)==SET)
    {
        USART_ClearFlag(UART7, USART_IT_RXNE);
    }
}

#if 0
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

#endif
//�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(UART7, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(UART7, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(UART7);
}


/*------------------------------------test------------------------------------*/




