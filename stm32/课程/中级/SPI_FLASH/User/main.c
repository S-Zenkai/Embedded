#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_spi_flash.h"
#if 1
uint8_t WriteBuff[]={"hello world"};
#define ArrSize sizeof(WriteBuff)/sizeof(WriteBuff[0])-1
uint8_t ReBuff[ArrSize]={3};
int main(void)
{
	uint32_t i=0;
//	uint8_t ReBuff[4096]={3};
//	for(i=0;i<4096;i++)
//	{
//		ReBuff[i]=i;
//	}
//	for(i=0;i<ArrSize;i++)
//	{
//		WriteBuff[i]=i%15;
//	}
	uint32_t RecData;
	USART_Config();
	SPI_Flash_Init();
	SPI_Flash_ReadID(&RecData);
	//printf("Manufacturer:%x\nMemory Type:%x\nCapacity:%x",
	//(RecData&0xFF0000)>>16,(RecData&0xFF00)>>8,(RecData&0xFF));
	printf("ID:%x\n",RecData);
	SPI_Flash_Sector_Erase(Flash_Sector_Erase_Addr);
	SPI_Flash_Waitfor();
//	SPI_Flash_Read_Data(Flash_Read_Data_Addr,ReBuff,4096);
//	printf("擦除后的扇区：");
//	for(i=0;i<4096;i++)
//	{
//		if(i%10==0)
//			printf("\n");
//		printf("%x ",ReBuff[i]);
//	}
//	SPI_Flash_Page_Program(Flash_Page_Program_Addr,WriteBuff,256);
	SPI_FLASH_BufferWrite(Flash_Page_Program_Addr, WriteBuff,ArrSize);
	SPI_Flash_Waitfor();
	SPI_Flash_Read_Data(Flash_Page_Program_Addr,ReBuff,ArrSize);
	printf("写入的数据：");
	printf("\n");
	printf("%s\n",ReBuff);
	for(i=0;i<ArrSize;i++)
	{
		if(ReBuff[i]!=WriteBuff[i])
			break;
	}
	if(i==ArrSize)
		printf("win\n");
	while(1);
}


#endif


