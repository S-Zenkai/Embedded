#include "bsp_i2c_ee.h"


//I2C初始化
void I2C_EE_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	//开时钟
	RCC_I2C_GPIO_PeriphClockCmd(I2C_SCL_GPIO_CLC|I2C_SDA_GPIO_CLC, ENABLE);
	RCC_I2C_PeriphClockCmd(I2Cx_CLC, ENABLE);
	
	//GPIO初始化
	GPIO_InitStructure.GPIO_Pin=I2C_SCL_GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(I2C_SCL_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA_GPIO_Pin;
	GPIO_Init(I2C_SDA_GPIO, &GPIO_InitStructure);
	
	//I2C初始化
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed=400000;//有疑问
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1=I2Cx_OwnAddr;
	I2C_Init(I2Cx_EE, &I2C_InitStructure);
	I2C_Cmd(I2Cx_EE,ENABLE);
}


//单个字节写入
uint8_t I2C_EE_Byte_Write(uint32_t I2Cx_Word_Addr, uint8_t Data)
{
	uint16_t Count=Event_Waiting_Time;
	I2C_GenerateSTART(I2Cx_EE, ENABLE);
	while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS)
	{
		Count--;
	}
	if(Count==0)
	{
		I2C_TIMEOUT_UserCallback(1);
		return 1;
	}
	I2C_Send7bitAddress(I2Cx_EE, I2Cx_Slave_Addr, I2C_Direction_Transmitter);
	Count=Event_Waiting_Time;
	while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS)
	{
		Count--;
	}
	if(Count==0)
	{
		I2C_TIMEOUT_UserCallback(2);
		return 1;
	}
	I2C_SendData(I2Cx_EE, I2Cx_Word_Addr);
	Count=Event_Waiting_Time;
	while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTING)!=SUCCESS)
	{
		Count--;
	}
	if(Count==0)
	{
		I2C_TIMEOUT_UserCallback(3);
		return 1;
	}
	I2C_SendData(I2Cx_EE, Data);
	Count=Event_Waiting_Time;
	while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS)
	{
		Count--;
	}
	if(Count==0)
	{
		I2C_TIMEOUT_UserCallback(4);
		return 1;
	}
	I2C_GenerateSTOP(I2Cx_EE, ENABLE);
	return 0;
}


//多字节写入
uint8_t I2C_EE_Page_Write(uint32_t I2Cx_Word_Addr, uint8_t* Data,uint8_t DataNum)
{
	uint16_t Count=Event_Waiting_Time;
	I2C_GenerateSTART(I2Cx_EE, ENABLE);
	while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS)
	{
		Count--;
	}
	if(Count==0)
	{
		I2C_TIMEOUT_UserCallback(1);
		return 1;
	}
	I2C_Send7bitAddress(I2Cx_EE, I2Cx_Slave_Addr, I2C_Direction_Transmitter);
	Count=Event_Waiting_Time;
	while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS)
	{
		Count--;
	}
	if(Count==0)
	{
		I2C_TIMEOUT_UserCallback(2);
		return 1;
	}
	I2C_SendData(I2Cx_EE, I2Cx_Word_Addr);
	Count=Event_Waiting_Time;
	while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTING)!=SUCCESS)
	{
		Count--;
	}
	if(Count==0)
	{
		I2C_TIMEOUT_UserCallback(3);
		return 1;
	}
	while(DataNum!=0)
	{
		I2C_SendData(I2Cx_EE, *Data);
	  Count=Event_Waiting_Time;
	  while(Count!=0&&I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS)
	  {
	  	Count--;
	  }
	  if(Count==0)
	  {
	  	I2C_TIMEOUT_UserCallback(4);
	   	return 1;
	  }
		Data++;//数组越界怎么处理
		DataNum--;
	}
	I2C_GenerateSTOP(I2Cx_EE, ENABLE);
	return 0;
}


uint8_t I2C_EE_BufferWrite(uint32_t I2Cx_Word_Addr, uint8_t* Data,uint8_t DataNum)
{
	uint8_t addr,PageNum,DataSingle,count;
	addr=I2Cx_Word_Addr%EEPROM_PageByteNum;
	PageNum=DataNum/EEPROM_PageByteNum;
	DataSingle=DataNum%EEPROM_PageByteNum;
	count=EEPROM_PageByteNum-addr;
	//如果地址对齐
	if(addr==0)
	{
		//输入数据不足一页
		if (PageNum==0)
		{
			I2C_EE_Page_Write(I2Cx_Word_Addr, Data,DataNum);
			Wait_Write();//还需要等待写入时序吗？？
		}
		//输入大于一页
		else
		{
			//数据正好填满每一页
			if(DataSingle==0)
			{
				while(PageNum--)
				{
					I2C_EE_Page_Write(I2Cx_Word_Addr, Data,EEPROM_PageByteNum);
		    	Wait_Write();
					I2Cx_Word_Addr=I2Cx_Word_Addr+EEPROM_PageByteNum;
					Data=Data+EEPROM_PageByteNum;
				}
			}
			else
			{
				while(PageNum--)
				{
					I2C_EE_Page_Write(I2Cx_Word_Addr, Data,EEPROM_PageByteNum);
		    	Wait_Write();
					I2Cx_Word_Addr=I2Cx_Word_Addr+EEPROM_PageByteNum;
					Data=Data+EEPROM_PageByteNum;
				}
				I2C_EE_Page_Write(I2Cx_Word_Addr, Data,DataSingle);
				Wait_Write();
			}
		}
	}
	//地址不对齐
	else
	{
		I2C_EE_Page_Write(I2Cx_Word_Addr, Data,count);
		Wait_Write();
		DataNum=DataNum-count;
		PageNum=DataNum/EEPROM_PageByteNum;
	  DataSingle=DataNum%EEPROM_PageByteNum;
		Data=Data+count;
		I2Cx_Word_Addr=I2Cx_Word_Addr+count;
		if (PageNum==0)
		{
			I2C_EE_Page_Write(I2Cx_Word_Addr, Data,DataNum);
			Wait_Write();//还需要等待写入时序吗？？
		}
		//输入大于一页
		else
		{
			//数据正好填满每一页
			if(DataSingle==0)
			{
				while(PageNum--)
				{
					I2C_EE_Page_Write(I2Cx_Word_Addr, Data,EEPROM_PageByteNum);
		    	Wait_Write();
					I2Cx_Word_Addr=I2Cx_Word_Addr+EEPROM_PageByteNum;
					Data=Data+EEPROM_PageByteNum;
				}
			}
			else
			{
				while(PageNum--)
				{
					I2C_EE_Page_Write(I2Cx_Word_Addr, Data,EEPROM_PageByteNum);
		    	Wait_Write();
					I2Cx_Word_Addr=I2Cx_Word_Addr+EEPROM_PageByteNum;
					Data=Data+EEPROM_PageByteNum;
				}
				I2C_EE_Page_Write(I2Cx_Word_Addr, Data,DataSingle);
				Wait_Write();
			}
		}
	}
}




void I2C_EE_Random_Read(uint32_t I2Cx_Word_Addr,uint8_t* ReceiverData)
{
	uint16_t Count=Event_Waiting_Time;
	I2C_GenerateSTART(I2Cx_EE, ENABLE);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);
	I2C_Send7bitAddress(I2Cx_EE, I2Cx_Slave_Addr, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS);
	I2C_SendData(I2Cx_EE, I2Cx_Word_Addr);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTING)!=SUCCESS);
	I2C_GenerateSTART(I2Cx_EE, ENABLE);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);
	I2C_Send7bitAddress(I2Cx_EE, I2Cx_Slave_Addr, I2C_Direction_Receiver);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!=SUCCESS);
	I2C_AcknowledgeConfig(I2Cx_EE, DISABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
	*ReceiverData=I2C_ReceiveData(I2Cx_EE);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_RECEIVED)!=SUCCESS);
	I2C_GenerateSTOP(I2Cx_EE, ENABLE);
	I2C_AcknowledgeConfig(I2Cx_EE, ENABLE);
}


void I2C_EE_Sequential_Read(uint32_t I2Cx_Word_Addr,uint8_t* ReceiverData,uint8_t DataNum)
{
	uint16_t Count=Event_Waiting_Time;
	I2C_GenerateSTART(I2Cx_EE, ENABLE);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);
	I2C_Send7bitAddress(I2Cx_EE, I2Cx_Slave_Addr, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS);
	I2C_SendData(I2Cx_EE, I2Cx_Word_Addr);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTING)!=SUCCESS);
	I2C_GenerateSTART(I2Cx_EE, ENABLE);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);
	I2C_Send7bitAddress(I2Cx_EE, I2Cx_Slave_Addr, I2C_Direction_Receiver);
	while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!=SUCCESS);
	while(DataNum!=0)
	{
		if(DataNum==1)
		{
			I2C_AcknowledgeConfig(I2Cx_EE, DISABLE);
		}
		while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
		*ReceiverData=I2C_ReceiveData(I2Cx_EE);
//	  while(I2C_CheckEvent(I2Cx_EE, I2C_EVENT_MASTER_BYTE_RECEIVED)!=SUCCESS);
		ReceiverData++;
		DataNum--;
	}
	I2C_GenerateSTOP(I2Cx_EE, ENABLE);
	I2C_AcknowledgeConfig(I2Cx_EE, ENABLE);
}


void Wait_Write(void)
{
	do
	{
		I2C_GenerateSTART(I2Cx_EE, ENABLE);
		while(I2C_GetFlagStatus(I2Cx_EE, I2C_FLAG_SB)==RESET);
		I2C_Send7bitAddress(I2Cx_EE, I2Cx_Slave_Addr, I2C_Direction_Transmitter);
	}while(I2C_GetFlagStatus(I2Cx_EE, I2C_FLAG_ADDR)==RESET);
	I2C_GenerateSTOP(I2Cx_EE, ENABLE);
}



static  uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* Block communication and all processes */
  EEPROM_ERROR("I2C 等待超时!errorCode = %d",errorCode);
  
  return 0;
}


