/**
 ****************************************************************************************************
 * @file        bsp_i2c.c
 * @author      
 * @version     
 * @date        2024-09-10
 * @brief       ģ��IICʵ��
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:stm32f103vet6
 *
 ****************************************************************************************************
 */
 
 
#include "./I2C_Software/bsp_i2c.h"



/**
 *  @brief  �ȴ���gpio�������
 *  �����ʱ����ͨ���߼������ǲ��Եõ��ġ�
 *  ����������CPU��Ƶ72MHz ��MDK���뻷����1���Ż�
 *
 *	ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
 *	ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
 * 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
 *  @param   ��
 *  @retval  ��
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
 *  @brief   ������ʼ�ź� 
 *  @param   ��
 *  @retval  ��
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
 *  @brief   ����ֹͣ�ź� 
 *  @param   ��
 *  @retval  ��
 */
void I2C_Stop(void)
{
	I2C_SDA_Reset();
	I2C_SCL_Set();
	I2C_Delay();
	I2C_SDA_Set();
}

/**
 *  @brief   ���ն˲���Ӧ���ź� 
 *  @param   ��
 *  @retval  ��
 */
void I2C_Ack(void)
{
	I2C_SDA_Reset();
	I2C_Delay();
	I2C_SCL_Set();
	I2C_Delay();
	I2C_SCL_Reset();
	I2C_Delay();
	I2C_SDA_Set();//����ΪʲôҪ�ͷ�SDA����
}

/**
 *  @brief   ���ն˲�����Ӧ���ź� 
 *  @param   ��
 *  @retval  ��
 */
void I2C_NAck(void)
{
	I2C_SDA_Set();
	I2C_Delay();
	I2C_SCL_Set();//����һ��ʱ������
	I2C_Delay();
	I2C_SCL_Reset();
	I2C_Delay();//����Ϊʲô�����ͷ�SDA����Ȩ
}

/**
 *  @brief   �ȴ�Ӧ���ź� 
 *  @param   ��
 *  @retval  ret:0��ʾӦ���źţ�1��ʾ��Ӧ���ź�
 */
uint8_t I2C_WaitAck(void)
{
	uint8_t ret;
	I2C_SDA_Set();//�ͷ�SDA����Ȩ
	I2C_SCL_Set();
	I2C_Delay();//��ʱ���ն˻ᷢ��Ӧ���ź�
	if(I2C_SDA_Read())
		ret=1;
	else
		ret=0;
	I2C_SCL_Reset();
	I2C_Delay();
	return ret;
}

/**
 *  @brief   дһ���ֽ� 
 *  @param   data:Ҫд����ֽ�
 *  @retval  ��
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
		I2C_Delay();//SDA��ÿ��ʱ�����ڴ���һλ����
		I2C_SCL_Set();
	  I2C_Delay();
		I2C_SCL_Reset();
  	I2C_Delay();
		data=data<<1;
	}
	I2C_SDA_Set();//�ͷ����ߣ�����Ϊʲô����Ҫʱ�����ڣ�
	I2C_Delay();
}

/**
 *  @brief   дһ���ֽ� 
 *  @param   ��
 *  @retval  ��ȡ�����ֽ�
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
 *  @brief   ���ͨѶ�Ƿ����� 
 *  @param   addr���豸��ַ
 *  @retval  1��ͨѶ�쳣��0��ͨѶ����
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

