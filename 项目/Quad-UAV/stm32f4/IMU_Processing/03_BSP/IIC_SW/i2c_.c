/**
 * *****************************************************************************
 * @file        bsp_iic.c
 * @brief       
 * @author      
 * @date        2024-12-05
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:    stm32f103vet6
 * 
 * *****************************************************************************
 */
/*----------------------------------include-----------------------------------*/
#include "i2c_sw.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*----------------------------------function----------------------------------*/
static void IIC_Delay(void)
{
    uint8_t i;
    /*��
        �����ʱ����ͨ���߼������ǲ��Եõ��ġ�
        ����������CPU��Ƶ72MHz ��MDK���뻷����1���Ż�

        ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz
        ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us
        ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us
        (������Ұ������Ľ�����������߼������ǲ���Ľ������һ��)
    */
    for (i = 0; i < 10; i++)
        ;
}

/**
 * @brief       SCL��SDA���ų�ʼ����SCL(PB12),SDA(PB13)
 * 
 */
static void IIC_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /*SCL(PB8)*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*SDA(PB9)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/**
 * @brief       iic���ų�ʼ��
 * 
 */
void iic_init(void)
{
    IIC_GPIO_Configure();
    IIC_SCL_WH;
    IIC_SDA_WH;
}

/**
 * @brief       ��ʼ�źţ�SCL�ߵ�ƽʱ��SDA�ߵ�ƽ�л�Ϊ�͵�ƽ
 * 
 */
void IIC_Start(void)
{
    IIC_SDA_WH;
    IIC_SCL_WH;
    IIC_Delay();
    IIC_SDA_WL;
    IIC_Delay();
    IIC_SCL_WL;
    IIC_Delay();
}

/**
 * @brief       ����Ӧ���źţ��ڽ�����һ���ֽں����һ��ʱ�ӷ��ͣ�0ΪӦ��1Ϊ��Ӧ��
 *
 * @param       ack
 *                  IIC_ASK:Ӧ��
 *                  IIC_NASK:��Ӧ��
 */
void IIC_SendAck(uint8_t ack)
{
    if (ack == IIC_ASK)
    {
        IIC_SDA_WL;
    }
    else /*ack==IIC_NASK*/
    {
        IIC_SDA_WH;
    }
    IIC_Delay();
    IIC_SCL_WH;
    IIC_Delay();
    IIC_SCL_WL;
    IIC_Delay();
    if(ack==IIC_NASK)
    {
        IIC_SDA_WH;
    }
}

/**
 * @brief       ����Ӧ���źţ�������һ���ֽں��һ��ʱ�ӽ��գ�����ǰ��Ҫ�ͷ�SDA���ߣ�
 *
 * @return      uint8_t     IIC_ASK:Ӧ��
 *                          IIC_NASK:��Ӧ��
 */
uint8_t IIC_ReceiveAck(void)
{
    uint8_t ack;
    // IIC_SDA_WH;
    // IIC_Delay();
    IIC_SCL_WH;
    IIC_Delay();
    if (IIC_SDA_R == 0)
    {
        ack = IIC_ASK;
    }
    else /*ack==IIC_NASK*/
    {
        ack = IIC_NASK;
    }
    IIC_SCL_WL;
    IIC_Delay();
    return ack;
}

/**
 * @brief       ����һ���ֽڣ�������Ϻ����ͷ�SDA����
 * 
 * @param       data    Ҫ���͵�����
 */
void IIC_SendByte(uint8_t data)
{
    uint8_t i = 0;
    for (i = 0; i < 8;i++)
    {
        if(data&(0x80>>i))
        {
            IIC_SDA_WH;
        }
        else
        {
            IIC_SDA_WL;
        }
        IIC_Delay();/*����Ҫ��*/
        IIC_SCL_WH;
        IIC_Delay();
        IIC_SCL_WL;
        if (i == 7)
        {
            IIC_SDA_WH; /*�ͷ�SDA����*/
        }
        IIC_Delay();
    }
}

/**
 * @brief       ����һ���ֽ�
 * 
 * @return      uint8_t 
 */
uint8_t IIC_ReceiveByte(void)
{
    uint8_t i = 0;
    uint8_t data;
    IIC_SCL_WH;
    IIC_Delay();
    for (i = 0; i < 8; i++)
    {
        IIC_SCL_WH;
        IIC_Delay();
        if (IIC_SDA_R == 1)
        {
            data = data | (1 << (7 - i));
        }
        else
        {
            data = data & (~(1 << (7 - i)));
        }
        // IIC_Delay();
        IIC_SCL_WL;
        IIC_Delay();
    }
    return data;
}

/**
 * @brief       ��ֹ�ź�
 * 
 */
void IIC_Stop(void)
{
    IIC_SDA_WL;
    IIC_Delay();
    IIC_SCL_WH;
    IIC_Delay();
    IIC_SDA_WH;
    IIC_Delay();
}

/*------------------------------------test------------------------------------*/
uint8_t test(void)
{
    uint8_t ret=5;
    IIC_Start();
    IIC_SendByte(0xD1);
    ret = IIC_ReceiveAck();
    IIC_Stop();
    return ret;
}
