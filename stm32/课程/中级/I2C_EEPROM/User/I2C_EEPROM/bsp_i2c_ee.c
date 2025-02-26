#include "bsp_i2c_ee.h"
#include "bsp_usart.h"


void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStrructure;
	
	//��ʱ��
	I2C_EEPROM_GPIO_CLC_FUN(I2C_EEPROM_GPIO_CLC, ENABLE);
	I2C_EEPROM_CLC_FUN(I2C_EEPROM_CLC, ENABLE);
	
	//GPIO����
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin=I2C_EEPROM_SCL_GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(I2C_EEPROM_SCL_GPIO_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=I2C_EEPROM_SDA_GPIO_Pin;
	GPIO_Init(I2C_EEPROM_SDA_GPIO_Port, &GPIO_InitStructure);

	//I2C����
	I2C_InitStrructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStrructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStrructure.I2C_ClockSpeed=I2C_Speed;
	I2C_InitStrructure.I2C_DutyCycle=I2C_DutyCycle_16_9;
	I2C_InitStrructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStrructure.I2C_OwnAddress1=I2C_Host_Addr;
	I2C_Init(I2C1,&I2C_InitStrructure);
	I2C_Cmd(I2C1, ENABLE);
}

#if 0
void I2C_EE_SendByte(uint8_t S_Addr,uint8_t Data)
{
	I2C_GenerateSTART(I2C1, ENABLE);  
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);//����Ӧ�ü���ĸ��¼�,Ӧ����EV6
	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);
	I2C_SendData(I2C1, Data);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);//����Ӧ����EV8_2
	I2C_GenerateSTOP(I2C1, ENABLE);
}

#elif 0

void I2C_EE_SendByte(uint8_t S_Addr,uint8_t Data)
{
	I2C_GenerateSTART(I2C1, ENABLE);  
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);//����Ӧ�ü���ĸ��¼�,Ӧ����EV6
	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);
	I2C_SendData(I2C1, Data);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);//����Ӧ����EV8_2
	I2C_GenerateSTOP(I2C1, ENABLE);
}

#elif 1


static __IO uint32_t  I2C_EE_Counter;
uint32_t I2C_EE_WriteByte(uint8_t S_Addr,uint8_t Data)
{
	I2C_EE_Counter=Time_Wait_EV;
	I2C_GenerateSTART(I2C1, ENABLE);
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(0);
	}
	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(1);
	}
	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(2);
	}
	I2C_SendData(I2C1, Data);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(3);
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 1;
}


uint32_t I2C_EE_PageWrite(uint8_t S_Addr,uint8_t* Data,uint32_t Data_Size)
{
	I2C_EE_Counter=Time_Wait_EV;
	I2C_GenerateSTART(I2C1, ENABLE);
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(0);
	}
	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(1);
	}
	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(2);
	}
	while(Data_Size!=0)
	{
		I2C_SendData(I2C1, *Data);
	  I2C_EE_Counter=Time_Wait_EV;
	  while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR)
	  {
		  I2C_EE_Counter--;
		  if (I2C_EE_Counter==0)
			  return I2C_TIMEOUT_UserCallback(3);
	  }
		Data_Size--;
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 1;
}


void EEPROM_Page_Write(uint8_t addr,uint8_t *data,uint8_t numByteToWrite)
{
	//������ʼ�ź�
	I2C_GenerateSTART(I2C1,ENABLE);
	
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//EV5�¼�����⵽�������豸��ַ
	I2C_Send7bitAddress(I2C1,I2C_Slava_Addr,I2C_Direction_Transmitter);
	
  while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) == ERROR);

	//EV6�¼�����⵽������Ҫ�����Ĵ洢��Ԫ��ַ
	I2C_SendData (I2C1,addr);
	
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING ) == ERROR);

	
	while(numByteToWrite)
	{
		//EV8�¼�����⵽������Ҫ�洢������
		I2C_SendData (I2C1,*data);
		
		while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED ) == ERROR);
		
		data++;
		numByteToWrite--;
			
	}
	//���ݴ������
	I2C_GenerateSTOP(I2C1,ENABLE);	

}


//void I2C_EE_SendByte(uint8_t S_Addr,uint8_t Data)
//{
//	uint16_t Count=Count_Wait_EV;
//	I2C_GenerateSTART(I2C1, ENABLE);  
//  /* Test on EV5 and clear it */
//  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
//  {
//		Count--;
//    if(Count== 0)
//		{			
//			I2C_TIMEOUT_UserCallback(0);
//			break;
//		}
//  }
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
////	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_MODE_SELECT,I2C_CheckEvent,0);
//	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);//����Ӧ�ü���ĸ��¼�,Ӧ����EV6
////	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED,I2C_CheckEvent,1);
//	while(!I2C_CheckEvent(I2C1 , I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);
////	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING,I2C_CheckEvent,2);
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	I2C_SendData(I2C1, Data);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR);//����Ӧ����EV8_2
////	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED,I2C_CheckEvent,3);
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	I2C_GenerateSTOP(I2C1, ENABLE);
//}


uint32_t I2C_EE_ReadByte(uint8_t S_Addr,uint8_t* Data)
{
	I2C_GenerateSTART(I2C1, ENABLE);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(4);
	}
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
  I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(5);
	}
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);//����Ӧ�ü���ĸ��¼�
	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(6);
	}
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);//����Ӧ�ü��EV8�¼�����Ϊ�Ƿ�������
	I2C_GenerateSTART(I2C1, ENABLE);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(7);
	}
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Receiver);//�������
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(8);
	}
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);//����Ӧ�ü���ĸ��¼�
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR);//ע�⣬EV6�ڿ�����2����һ������һ������
	I2C_AcknowledgeConfig (I2C1,DISABLE);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(9);
	}
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED ) == ERROR);
	*Data=I2C_ReceiveData(I2C1);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(10);
	}
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 1;
}



uint32_t I2C_EE_SequentialRead(uint8_t S_Addr,uint8_t* Data,uint32_t Data_Size)
{
	uint16_t i=0;
	I2C_GenerateSTART(I2C1, ENABLE);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(4);
	}
  I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(5);
	}
	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(6);
	}
	I2C_GenerateSTART(I2C1, ENABLE);
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(7);
	}
	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Receiver);//�������
	I2C_EE_Counter=Time_Wait_EV;
	while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR)
	{
		I2C_EE_Counter--;
		if (I2C_EE_Counter==0)
			return I2C_TIMEOUT_UserCallback(8);
	}
	i=0;
	while(Data_Size!=0)
	{
		if(Data_Size==1)
		{
			I2C_AcknowledgeConfig (I2C1,DISABLE);
	    I2C_EE_Counter=Time_Wait_EV;
	    while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR)
	    {
		    I2C_EE_Counter--;
		    if (I2C_EE_Counter==0)
		    	return I2C_TIMEOUT_UserCallback(9);
	    }
		}
	  *(Data+i)=I2C_ReceiveData(I2C1);
	  I2C_EE_Counter=Time_Wait_EV;
	  while(I2C_EE_Counter>0 && I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR)
	  {
	  	I2C_EE_Counter--;
	  	if (I2C_EE_Counter==0)
	  		return I2C_TIMEOUT_UserCallback(10);
	  }
		Data_Size--;
		i++;
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 1;
}


void EEPROM_Read(uint8_t addr,uint8_t *data,uint8_t numByteToRead)
{
	//������ʼ�ź�
	I2C_GenerateSTART(I2C1,ENABLE);
	
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//EV5�¼�����⵽�������豸��ַ
	I2C_Send7bitAddress(I2C1,I2C_Slava_Addr,I2C_Direction_Transmitter);
	
  while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) == ERROR);

	//EV6�¼�����⵽������Ҫ�����Ĵ洢��Ԫ��ַ
	I2C_SendData (I2C1,addr);
	
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING ) == ERROR);
	
	//�ڶ�����ʼ�ź�
	//������ʼ�ź�
	I2C_GenerateSTART(I2C1,ENABLE);
	
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//EV5�¼�����⵽�������豸��ַ
	I2C_Send7bitAddress(I2C1,I2C_Slava_Addr,I2C_Direction_Receiver);
	
  while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) == ERROR);


	while(numByteToRead)
	{
		if(numByteToRead == 1)
		{		
			//���Ϊ���һ���ֽ�
			I2C_AcknowledgeConfig (I2C1,DISABLE);
		}		
		
		//EV7�¼�����⵽	
		while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED ) == ERROR);

		//EV7�¼�����⵽�������ݼĴ������µ���Ч����	
		*data = I2C_ReceiveData(I2C1);
		
		data++;
		
		numByteToRead--;
		
	}
	
	
	//���ݴ������
	I2C_GenerateSTOP(I2C1,ENABLE);	
	
	//��������ACKʹ�ܣ��Ա��´�ͨѶ
	I2C_AcknowledgeConfig (I2C1,ENABLE);

}


//uint8_t Data;
//void I2C_EE_ReadByte(uint8_t S_Addr,uint8_t* Data)
//{
//	//uint16_t count=1000;
//	I2C_GenerateSTART(I2C1, ENABLE);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
////		;
//	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_MODE_SELECT,I2C_CheckEvent,3);
//  I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);//����Ӧ�ü���ĸ��¼�
//	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED,I2C_CheckEvent,3);
//	I2C_SendData(I2C1, I2C_Slava_Wri_Addr);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==ERROR);//����Ӧ�ü��EV8�¼�����Ϊ�Ƿ�������
//	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING,I2C_CheckEvent,3);
//	I2C_GenerateSTART(I2C1, ENABLE);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
////		;
//	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_MODE_SELECT,I2C_CheckEvent,3);
////  I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Transmitter);
//	I2C_Send7bitAddress(I2C1, S_Addr, I2C_Direction_Receiver);//�������
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);//����Ӧ�ü���ĸ��¼�
//	Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED,I2C_CheckEvent,3);
//	I2C_AcknowledgeConfig (I2C1,DISABLE);
////	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED ) == ERROR);
//  Wait_EV(Count_Wait_EV,I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED,I2C_CheckEvent,3);
//	*Data=I2C_ReceiveData(I2C1);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
////	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR);
//	//I2C_AcknowledgeConfig(I2C1, DISABLE);
//	I2C_GenerateSTOP(I2C1, ENABLE);
//	I2C_AcknowledgeConfig(I2C1, ENABLE);
//}

#if 0
uint32_t Wait_EV(uint32_t I2C_EVENT,ErrorStatus (*Wait_EV_I2C_CheckEvent)(I2C_TypeDef* , uint32_t),uint8_t ErrorCode)
{
	uint16_t I2C_EE_Counter=Time_Wait_EV;
	while(Wait_EV_I2C_CheckEvent(I2C1, I2C_EVENT)==ERROR&&I2C_EE_Counter!=0)
	{
		I2C_EE_Counter--;
	}
	if(I2C_EE_Counter==0)
	{
		return I2C_TIMEOUT_UserCallback(ErrorCode);
	}
}
#endif
//��������͸���Դ�벻һ��������д��������
//void Wait_Write_Complete(void)
//{
//	do
//	{
//		I2C_GenerateSTART(I2C1, ENABLE);
//	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
//		;
//  I2C_Send7bitAddress(I2C1, I2C_Slava_Addr, I2C_Direction_Transmitter);
//	}while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
//	I2C_GenerateSTOP(I2C1, ENABLE);
//}


void Wait_Write_Complete(void)
{
	do
	{
		//������ʼ�ź�
		I2C_GenerateSTART(I2C1,ENABLE);
		
		while(I2C_GetFlagStatus (I2C1,I2C_FLAG_SB) == RESET);
		
		//EV5�¼�����⵽�������豸��ַ
		I2C_Send7bitAddress(I2C1,I2C_Slava_Addr,I2C_Direction_Transmitter);
	}  
	while(I2C_GetFlagStatus (I2C1,I2C_FLAG_ADDR) == RESET);

	//EEPROM�ڲ�ʱ����ɴ������
	I2C_GenerateSTOP(I2C1,ENABLE);	
}




static  uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* Block communication and all processes */
  EEPROM_ERROR("I2C �ȴ���ʱ!errorCode = %d",errorCode);
  
  return 0;
}


#endif
