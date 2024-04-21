#include "bsp_i2c.h"

 /**
  * @brief  IIC EEPROM  配置,工作参数配置
  * @param  无
  * @retval 无
  */
void I2C_EE_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure; 

	// 打开IIC GPIO的时钟
	EEPROM_I2C_GPIO_APBxClkCmd(EEPROM_I2C_SCL_GPIO_CLK|EEPROM_I2C_SDA_GPIO_CLK, ENABLE);
	
	// 打开IIC 外设的时钟
	EEPROM_I2C_APBxClkCmd(EEPROM_I2C_CLK, ENABLE);

	// 将IIC SCL的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = EEPROM_I2C_SCL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(EEPROM_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  // 将IIC SDA的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = EEPROM_I2C_SDA_GPIO_PIN;
	GPIO_Init(EEPROM_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
	
	// 配置IIC的工作参数
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable  ;//使能应答
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit ;//使用7位地址模式
	I2C_InitStructure.I2C_ClockSpeed = EEPROM_I2_BAUDRATE; //配置SCL时钟频率
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2 ;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C ;
	I2C_InitStructure.I2C_OwnAddress1 = STM32_I2C_OWN_ADDR; //这是STM32 IIC自身设备地址，只要是总线上唯一即可
	
	I2C_Init(EEPROM_I2C,&I2C_InitStructure);
	
	// 使能串口
	I2C_Cmd (EEPROM_I2C, ENABLE);	    
}


//向EEPROM写入一个字节

void EEPROM_Byte_Write(uint8_t addr,uint8_t data)
{
	//产生起始信号
	I2C_GenerateSTART(EEPROM_I2C,ENABLE);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//EV5事件被检测到，发送设备地址
	I2C_Send7bitAddress(EEPROM_I2C,EEPROM_ADDR,I2C_Direction_Transmitter);
	
  while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) == ERROR);

	//EV6事件被检测到，发送要操作的存储单元地址
	I2C_SendData (EEPROM_I2C,addr);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTING ) == ERROR);

  //EV8事件被检测到，发送要存储的数据
	I2C_SendData (EEPROM_I2C,data);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED ) == ERROR);
	
	//数据传输完成
	I2C_GenerateSTOP(EEPROM_I2C,ENABLE);	

}



//向EEPROM写入多个字节（页写入），每次写入不能超过8个字节

void EEPROM_Page_Write(uint8_t addr,uint8_t *data,uint8_t numByteToWrite)
{
	//产生起始信号
	I2C_GenerateSTART(EEPROM_I2C,ENABLE);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//EV5事件被检测到，发送设备地址
	I2C_Send7bitAddress(EEPROM_I2C,EEPROM_ADDR,I2C_Direction_Transmitter);
	
  while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) == ERROR);

	//EV6事件被检测到，发送要操作的存储单元地址
	I2C_SendData (EEPROM_I2C,addr);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTING ) == ERROR);

	
	while(numByteToWrite)
	{
		//EV8事件被检测到，发送要存储的数据
		I2C_SendData (EEPROM_I2C,*data);
		
		while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED ) == ERROR);
		
		data++;
		numByteToWrite--;
			
	}
	//数据传输完成
	I2C_GenerateSTOP(EEPROM_I2C,ENABLE);	

}



//从EEPROM读取数据

void EEPROM_Read(uint8_t addr,uint8_t *data,uint8_t numByteToRead)
{
	//产生起始信号
	I2C_GenerateSTART(EEPROM_I2C,ENABLE);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//EV5事件被检测到，发送设备地址
	I2C_Send7bitAddress(EEPROM_I2C,EEPROM_ADDR,I2C_Direction_Transmitter);
	
  while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) == ERROR);

	//EV6事件被检测到，发送要操作的存储单元地址
	I2C_SendData (EEPROM_I2C,addr);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTING ) == ERROR);

	
	//第二次起始信号
	//产生起始信号
	I2C_GenerateSTART(EEPROM_I2C,ENABLE);
	
	while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//EV5事件被检测到，发送设备地址
	I2C_Send7bitAddress(EEPROM_I2C,EEPROM_ADDR,I2C_Direction_Receiver);
	
  while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) == ERROR);


	while(numByteToRead)
	{
		if(numByteToRead == 1)
		{		
			//如果为最后一个字节
			I2C_AcknowledgeConfig (EEPROM_I2C,DISABLE);
		}		
		
		//EV7事件被检测到	
		while(I2C_CheckEvent(EEPROM_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED ) == ERROR);

		//EV7事件被检测到，即数据寄存器有新的有效数据	
		*data = I2C_ReceiveData(EEPROM_I2C);
		
		data++;
		
		numByteToRead--;
		
	}
	
	
	//数据传输完成
	I2C_GenerateSTOP(EEPROM_I2C,ENABLE);	
	
	//重新配置ACK使能，以便下次通讯
	I2C_AcknowledgeConfig (EEPROM_I2C,ENABLE);

}


//等待EEPROM内部时序完成
void EEPROM_WaitForWriteEnd(void)
{
	
	do
	{
		//产生起始信号
		I2C_GenerateSTART(EEPROM_I2C,ENABLE);
		
		while(I2C_GetFlagStatus (EEPROM_I2C,I2C_FLAG_SB) == RESET);
		
		//EV5事件被检测到，发送设备地址
		I2C_Send7bitAddress(EEPROM_I2C,EEPROM_ADDR,I2C_Direction_Transmitter);
	}  
	while(I2C_GetFlagStatus (EEPROM_I2C,I2C_FLAG_ADDR) == RESET );

	//EEPROM内部时序完成传输完成
	I2C_GenerateSTOP(EEPROM_I2C,ENABLE);	
}

