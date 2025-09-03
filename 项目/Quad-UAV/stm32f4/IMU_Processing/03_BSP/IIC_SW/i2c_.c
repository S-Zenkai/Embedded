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
 * 实验平台:    stm32f103vet6
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
    /*　
        下面的时间是通过逻辑分析仪测试得到的。
        工作条件：CPU主频72MHz ，MDK编译环境，1级优化

        循环次数为10时，SCL频率 = 205KHz
        循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us
        循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us
        (上面是野火给出的结果，我是用逻辑分析仪测出的结果并不一样)
    */
    for (i = 0; i < 10; i++)
        ;
}

/**
 * @brief       SCL、SDA引脚初始化，SCL(PB12),SDA(PB13)
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
 * @brief       iic引脚初始化
 * 
 */
void iic_init(void)
{
    IIC_GPIO_Configure();
    IIC_SCL_WH;
    IIC_SDA_WH;
}

/**
 * @brief       起始信号，SCL高电平时，SDA高电平切换为低电平
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
 * @brief       发送应答信号，在接受完一个字节后的下一个时钟发送，0为应答，1为非应答
 *
 * @param       ack
 *                  IIC_ASK:应答
 *                  IIC_NASK:非应答
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
 * @brief       接受应答信号，接收完一个字节后的一个时钟接收（接受前需要释放SDA总线）
 *
 * @return      uint8_t     IIC_ASK:应答
 *                          IIC_NASK:非应答
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
 * @brief       发送一个字节，发送完毕后需释放SDA总线
 * 
 * @param       data    要发送的数据
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
        IIC_Delay();/*这里要吗？*/
        IIC_SCL_WH;
        IIC_Delay();
        IIC_SCL_WL;
        if (i == 7)
        {
            IIC_SDA_WH; /*释放SDA总线*/
        }
        IIC_Delay();
    }
}

/**
 * @brief       接收一个字节
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
 * @brief       终止信号
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
