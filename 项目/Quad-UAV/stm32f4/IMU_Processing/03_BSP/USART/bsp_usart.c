/**
 ******************************************************************************
 * @file    bsp_usart.c
 * @author  kai
 * @version V1
 * @date    2025/04/09
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
 * @brief  �ж����ȼ�����
 * @param  ��
 * @retval ��
 */
// static void USART_NVIC_Configure(void)
//{
//     NVIC_InitTypeDef NVIC_InitStructure;
//     /*���ȼ���*/
//     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); /*��������Ŀʹ��һ�μ���*/
//     /*USART1*/
//     NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//     NVIC_Init(&NVIC_InitStructure);
// }

/**************************************USART7����*********************************************/
/**
 * @brief  USART7_GPIO����
 * @note
 * @param  ��
 * @retval ��
 */
static void USART_DEBUG_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DEBUG_USART_GPIO_CLK_FUN(DEBUG_USART_GPIO_CLK, ENABLE);
    DEBUG_USART_CLK_FUN(DEBUG_USART_CLK, ENABLE); /*USART7��GPIOA*/
    GPIO_PinAFConfig(DEBUG_USART_TX_PORT, DEBUG_USART_AF_PinSource_TX, DEBUG_USART_AF);
    GPIO_PinAFConfig(DEBUG_USART_RX_PORT, DEBUG_USART_AF_PinSource_RX, DEBUG_USART_AF);
    /*��������*/
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

    /*USART����*/
    USART_InitStructure.USART_BaudRate = 115200;                                    /*������*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*Ӳ��������*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*���������ģʽ*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*��żУ��ģʽ*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*ֹͣλ*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*�����ֳ�*/
    USART_Init(DEBUGE_USART, &USART_InitStructure);
    USART_Cmd(DEBUGE_USART, ENABLE);
}

/**************************************UART4����*********************************************/

/**
 * @brief  TX-PA0    RX-PA1
 * @note
 * @param  ��
 * @retval ��
 */
static void UART4_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /*��������*/
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
    USART_InitStructure.USART_BaudRate = baudrate;                                  /*m8nĬ�ϲ�����Ϊ9600������������ѯ����������������m8n����ʵ������*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*Ӳ��������*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*���������ģʽ*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*��żУ��ģʽ*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*ֹͣλ*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*�����ֳ�*/
    USART_Init(UART4, &USART_InitStructure);
    // USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);/*��ʼ��ʹ��DMAת�����ݣ��ȵ�ƥ�䵽��ȷ�����ʺ���ʹ��*/
}

/**
 * @brief  ����USART4������
 * @note
 * @param  baudrate:������
 * @retval ��
 */
void USART4_SetBaudRate(uint32_t baud)
{
    /*����USART��DMA*/
    USART_DeInit(UART4);
    DMA_DeInit(DMA1_Stream2);

    /*����GPIO��USART��DMA*/
    UART4_GPIO_Configure();
    UART4_Configure(baud);

    /*ʹ��USART��DMA��DMA�ж�*/
    USART_Cmd(UART4, ENABLE);
    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Stream2, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
}

/**************************************USART7����*********************************************/
/**
 * @brief  USART7_GPIO����
 * @note
 * @param  ��
 * @retval ��
 */
static void USART_MAVLINK_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    MAVLINK_USART_GPIO_CLK_FUN(MAVLINK_USART_GPIO_CLK, ENABLE);
    MAVLINK_USART_CLK_FUN(MAVLINK_USART_CLK, ENABLE); /*USART7��GPIOA*/
    GPIO_PinAFConfig(MAVLINK_USART_TX_PORT, MAVLINK_USART_AF_PinSource_TX, MAVLINK_USART_AF);
    GPIO_PinAFConfig(MAVLINK_USART_RX_PORT, MAVLINK_USART_AF_PinSource_RX, MAVLINK_USART_AF);
    /*��������*/
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

    /*USART����*/
    USART_InitStructure.USART_BaudRate = 115200;                                    /*������*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*Ӳ��������*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 /*���������ģʽ*/
    USART_InitStructure.USART_Parity = USART_Parity_No;                             /*��żУ��ģʽ*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          /*ֹͣλ*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     /*�����ֳ�*/

    USART_Init(MAVLINK_USART, &USART_InitStructure);
    USART_DMACmd(MAVLINK_USART, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(MAVLINK_USART, ENABLE);
}

/**************************************USART6����*********************************************/
/**
 * @brief  USART6���ã�����˫��ͨ��
 * @note
 * @param  ��
 * @retval ��
 */
static void USART_ICC_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    ICC_USART_GPIO_CLK_FUN(ICC_USART_GPIO_CLK, ENABLE);
    ICC_USART_CLK_FUN(ICC_USART_CLK, ENABLE); /*USART7��GPIOA*/
    GPIO_PinAFConfig(ICC_USART_TX_PORT, ICC_USART_AF_PinSource_TX, ICC_USART_AF);
    GPIO_PinAFConfig(ICC_USART_RX_PORT, ICC_USART_AF_PinSource_RX, ICC_USART_AF);
    /*��������*/
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
/********************************************************************************************/

/**
 * @brief       USART��ʼ������
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

/**********************************************���ܺ���********************************************/
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
 * @note   ��Ҫ�����ú�������ӵȴ�������ɱ�־����ֹ���ݶ�ʧ
 *         ���轫���ݷ���ȫ�ֱ����У���ֹ����δ��ɣ��ֲ��������ͷţ��������ݴ���
 * @param  DMAy_Channelx: DMAͨ��
 * @param  buffer: ����ָ��
 * @param  len: ���ݳ���
 * @retval ��
 */
void USART_DMA_Send(DMA_Stream_TypeDef *DMAy_Streamx, uint8_t *buffer, uint16_t len)
{
    /*����ע�͵ĳ�����Ҫ���ⲿ���øú���ʱ��ӣ�����鷢����ɱ�־�Լ���Ҫ���͵����ݷ���ȫ�ֱ�����*/
    /*ȫ�ֱ�����Ϊ�˷�ֹ����δ��ɣ��ֲ��������ͷţ��������ݴ���*/
    // while (USART_Send_Done_Flag == RESET)
    // {
    // };
    // USART_Send_Done_Flag = RESET;
    //memcpy(ICC_TX_Buff, buffer, len);

    // ȷ��DMA�Ѿ��رգ�������������
    DMA_Cmd(DMAy_Streamx, DISABLE);
    // �ȴ�DMA���Ա�����
    while (DMA_GetCmdStatus(DMAy_Streamx) != DISABLE)
        ;
    // �����ڴ��ַ
    DMAy_Streamx->M0AR = (uint32_t)buffer;

    // ����Ҫ�����������
    DMA_SetCurrDataCounter(DMAy_Streamx, len);

    // ʹ��DMA Stream����ʼ����
    DMA_Cmd(DMAy_Streamx, ENABLE);
}

/**
 * @brief  ���ն���ֽڣ�֧�ֳ�ʱ���
 * @note   ��ʱʱ�����ԭ����Ҫ����ͨ��ģ�����ã��ɸ��ݲο��ֲ�ģ����Ӧʱ�����ã�
 * @param  USARTx: ���ھ��
 * @param  Data: ����ָ��
 * @param  length: ���ݳ���
 * @param  timeout: ��ʱʱ��
 * @retval ���ճɹ����ֽ���
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

//     /* ȡ���߰�λ */
//     temp_h = (ch & 0XFF00) >> 8;
//     /* ȡ���Ͱ�λ */
//     temp_l = ch & 0XFF;

//     /* ���͸߰�λ */
//     USART_SendData(pUSARTx, temp_h);
//     while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
//         ;

//     /* ���͵Ͱ�λ */
//     USART_SendData(pUSARTx, temp_l);
//     while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
//         ;
// }

// �ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
    /* ����һ���ֽ����ݵ����� */
    USART_SendData(UART7, (uint8_t)ch);

    /* �ȴ�������� */
    while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET)
        ;

    return (ch);
}

/// �ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
    /* �ȴ������������� */
    while (USART_GetFlagStatus(UART7, USART_FLAG_RXNE) == RESET)
        ;

    return (int)USART_ReceiveData(UART7);
}

/*------------------------------------test------------------------------------*/
