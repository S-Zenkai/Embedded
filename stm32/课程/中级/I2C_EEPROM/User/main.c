#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_i2c_ee.h"

#if 1

//写入数据长度
#define Size   8

int main(void)
{
	uint16_t i=0;
	uint8_t Data[Size]={0};
	uint8_t ByteData;
	uint8_t PageData[Size]={1,2,3};
	I2C_GPIO_Config();
	USART_Config();
	printf("写入的值：\n");
	for(i=0;i<Size;i++)
	{
		Data[i]=i;
		printf("%d ",Data[i]);
	}
//	printf("写入的值：\n");
	
#if 0
	//字写入与读取
	I2C_EE_WriteByte(I2C_Slava_Addr,8);
	Wait_Write_Complete();
	I2C_EE_ReadByte(I2C_Slava_Addr,&ByteData);
	printf("%d\n",ByteData);
#endif
#if 1
	//页写入与读取
	printf("读出的值\n");
//	I2C_EE_PageWrite(I2C_Slava_Addr,Data,Size);
	EEPROM_Page_Write(0,Data,8);
	Wait_Write_Complete();
	
	I2C_EE_SequentialRead(I2C_Slava_Addr,PageData,Size);
//	EEPROM_Read(0,PageData,8);
//	EEPROM_Read(8,&Data,1);
	for(i=0;i<Size;i++)
	{

		printf("%d ",PageData[i]);
	}
#endif
	while(1);
	
}

#elif 0

int main(void)
{
	uint16_t a=16;
	I2C_GPIO_Config();
	USART_Config();
	EEPROM_ERROR("0x%02X ", a);//关于%02X:0是填充符，如果输出的数值位数不够，则用0填充。2X:表示以两个字符输出16进制
  EEPROM_ERROR("错误EEPROM写入与读出的数据不一致\n\r");
}

#endif

