/**
 ****************************************************************************************************
 * @file        bsp_i2c.c
 * @author      
 * @version     
 * @date        2024-09-10
 * @brief       模拟IIC实验
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:stm32f103vet6
 *
 ****************************************************************************************************
 */
 
 
#include "./I2C_Software/bsp_i2c.h"



/**
 *  @brief  等待对gpio操作完成
 *  下面的时间是通过逻辑分析仪测试得到的。
 *  工作条件：CPU主频72MHz ，MDK编译环境，1级优化
 *
 *	循环次数为10时，SCL频率 = 205KHz 
 *	循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
 * 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
 *  @param   无
 *  @retval  无
 */
static void I2C_Delay(void)
{
	uint8_t i=0;
	for(i=0;i<10;i++);
}

void i2c_init(void)
{
	GPIO_InitTypeDef I2C_GPIO_Structure;
	I2C_GPIO_APBxClock_FUN(I2C_GPIO_SCK_CLK,ENABLE);
	//SCL
	I2C_GPIO_Structure.GPIO_Mode=GPIO_Mode_Out_OD;
	I2C_GPIO_Structure.GPIO_Pin=I2C_SCL_GPIO_Pin;
	I2C_GPIO_Structure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(I2C_SCL_GPIO,&I2C_GPIO_Structure);
	//SDA
	I2C_GPIO_Structure.GPIO_Pin=I2C_SDA_GPIO_Pin;
	GPIO_Init(I2C_SDA_GPIO,&I2C_GPIO_Structure);
}

/**
 *  @brief   产生起始信号 
 *  @param   无
 *  @retval  无
 */
void I2C_Start(void)
{
	I2C_SCL_Set();
	I2C_SDA_Set();
	I2C_Delay();
	I2C_SDA_Reset();
	I2C_Delay();
	I2C_SCL_Reset();
	I2C_Delay();
}

/**
 *  @brief   产生停止信号 
 *  @param   无
 *  @retval  无
 */
void I2C_Stop(void)
{
	I2C_SDA_Reset();
	I2C_SCL_Set();
	I2C_Delay();
	I2C_SDA_Set();
}

/**
 *  @brief   接收端产生应答信号 
 *  @param   无
 *  @retval  无
 */
void I2C_Ack(void)
{
	I2C_SDA_Reset();
	I2C_Delay();
	I2C_SCL_Set();
	I2C_Delay();
	I2C_SCL_Reset();
	I2C_Delay();
	I2C_SDA_Set();//这里为什么要释放SDA总线
}

/**
 *  @brief   接收端产生非应答信号 
 *  @param   无
 *  @retval  无
 */
void I2C_NAck(void)
{
	I2C_SDA_Set();
	I2C_Delay();
	I2C_SCL_Set();//产生一个时钟周期
	I2C_Delay();
	I2C_SCL_Reset();
	I2C_Delay();//这里为什么不用释放SDA控制权
}

/**
 *  @brief   等待应答信号 
 *  @param   无
 *  @retval  ret:0表示应答信号，1表示非应答信号
 */
uint8_t I2C_WaitAck(void)
{
	uint8_t ret;
	I2C_SDA_Set();//释放SDA控制权
	I2C_SCL_Set();
	I2C_Delay();//这时接收端会发送应答信号
	if(I2C_SDA_Read())
		ret=1;
	else
		ret=0;
	I2C_SCL_Reset();
	I2C_Delay();
	return ret;
}

/**
 *  @brief   写一个字节 
 *  @param   data:要写入的字节
 *  @retval  无
 */
void I2C_WriteByte(uint8_t data)
{
	uint8_t i=0;
	for(i=0;i<8;i++)
	{
		if(data&0x80)
		{
			I2C_SDA_Set();
		}
		else
			I2C_SDA_Reset();
		I2C_Delay();//SDA在每个时钟周期传输一位数据
		I2C_SCL_Set();
	  I2C_Delay();
		I2C_SCL_Reset();
  	I2C_Delay();
		data=data<<1;
	}
	I2C_SDA_Set();//释放总线，这里为什么不需要时钟周期？
	I2C_Delay();
}

/**
 *  @brief   写一个字节 
 *  @param   无
 *  @retval  读取到的字节
 */
uint8_t I2C_ReadByte(void)
{
	uint8_t i=0;
	uint8_t data=0;
	for(i=0;i<8;i++)
	{
		I2C_SCL_Set();
	  I2C_Delay();
		if(I2C_SDA_Read())
		{
			data++;
		}
		data=data<<1;
		I2C_SCL_Reset();
  	I2C_Delay();
	}
	return data;
}

/**
 *  @brief   检查通讯是否正常 
 *  @param   addr：设备地址
 *  @retval  1：通讯异常；0：通讯正常
 */
uint8_t CheckDevice(uint8_t addr)
{
	uint8_t ret;
	i2c_init();
	I2C_Start();
	I2C_WriteByte(addr);
	ret=I2C_WaitAck();
	I2C_Stop();
	return ret;
}

