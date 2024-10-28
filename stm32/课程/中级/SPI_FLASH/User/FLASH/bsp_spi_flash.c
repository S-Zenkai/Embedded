#include "bsp_spi_flash.h"

static  uint32_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

//GPIO初始化
static void SPI_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//开时钟
	FLASH_SPI_GPIO_APBxClock_FUN(FLASH_SPI_GPIO_CLK, ENABLE);
	
	//GPIO_SCK配置
	GPIO_InitStructure.GPIO_Pin=FLASH_SPI_GPIO_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_GPIO_SCK_PORT, &GPIO_InitStructure);
	
	//GPIO_MOSI配置
	GPIO_InitStructure.GPIO_Pin=FLASH_SPI_GPIO_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_GPIO_MOSI_PORT, &GPIO_InitStructure);
	
	//GPIO_MISO配置
	GPIO_InitStructure.GPIO_Pin=FLASH_SPI_GPIO_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_GPIO_MISO_PORT, &GPIO_InitStructure);
	
	//GPIO_CS配置
	GPIO_InitStructure.GPIO_Pin=FLASH_SPI_GPIO_CS_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_GPIO_CS_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
}


//SPI初始化
static void SPI_Config(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	
	//开时钟
	FLASH_SPI_APBxClock_FUN(FLASH_SPI_CLK, ENABLE);
	
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16;
	//选择mode0
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_High;
	SPI_InitStructure.SPI_CRCPolynomial=0;
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;
	SPI_Init(FLASH_SPIx, &SPI_InitStructure);
	SPI_Cmd(FLASH_SPIx,ENABLE);
}


void SPI_Flash_Init(void)
{
	SPI_GPIO_Config();
	SPI_Config();
}

//向Flash收发一个字节
uint16_t SPI_Flash_WritByte(uint8_t Data)
{
	uint16_t WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(FLASH_SPIx, SPI_I2S_FLAG_TXE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
			return SPI_TIMEOUT_UserCallback(0);
	}
	SPI_I2S_SendData(FLASH_SPIx, Data);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(FLASH_SPIx, SPI_I2S_FLAG_RXNE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
			return SPI_TIMEOUT_UserCallback(1);
	}
	return SPI_I2S_ReceiveData(FLASH_SPIx);
}



//读Flash设备ID
void SPI_Flash_ReadID(uint32_t* ReData)
{
	GPIO_ResetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
	SPI_Flash_WritByte(FLASH_Read_JEDEC_ID);
	*ReData=SPI_Flash_WritByte(FLASH_Invali_Data);
	*ReData <<= 8;
	*ReData |= SPI_Flash_WritByte(FLASH_Invali_Data);
	*ReData <<= 8;
	*ReData |= SPI_Flash_WritByte(FLASH_Invali_Data);
	GPIO_SetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
}

//写数据使能
void SPI_Flash_Write_Enable(void)
{
	GPIO_ResetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
	SPI_Flash_WritByte(Instruction_Write_Enable);
	GPIO_SetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
}

//Flash页擦除
void SPI_Flash_Sector_Erase(uint32_t Addr)
{
	SPI_Flash_Write_Enable();
	GPIO_ResetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
	SPI_Flash_WritByte(Instruction_Sector_Erase);
	SPI_Flash_WritByte((Addr>>16)&0xFF);
	SPI_Flash_WritByte((Addr>>8)&0xFF);
	SPI_Flash_WritByte((Addr)&0xFF);
	GPIO_SetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
}

//读Flash存储数据
void SPI_Flash_Read_Data(uint32_t Addr,uint8_t* ReData,uint32_t ByteNum)
{
	GPIO_ResetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
	SPI_Flash_WritByte(Instruction_Read_Data);
	SPI_Flash_WritByte((Addr>>16)&0xFF);
	SPI_Flash_WritByte((Addr>>8)&0xFF);
	SPI_Flash_WritByte((Addr)&0xFF);
	while(ByteNum--)
	{
		*ReData=SPI_Flash_WritByte(FLASH_Invali_Data);
		ReData++;
	}
	GPIO_SetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
}

//flash页写入
void SPI_Flash_Page_Program(uint32_t Addr,uint8_t* WriteBuff,uint32_t ByteNum)
{
	SPI_Flash_Write_Enable();
	GPIO_ResetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
	SPI_Flash_WritByte(Instruction_Page_Program);
	SPI_Flash_WritByte((Addr>>16)&0xFF);
	SPI_Flash_WritByte((Addr>>8)&0xFF);
	SPI_Flash_WritByte((Addr)&0xFF);
	while(ByteNum--)
	{
		SPI_Flash_WritByte(*WriteBuff);
		WriteBuff++;
	}
	GPIO_SetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
}



//flash写入数据，消除页写入一次只能256个数据,注意，写入数据必须是uint8_t程序才不用出问题
void SPI_FLASH_BufferWrite(uint32_t Addr, uint8_t* WriteBuff,uint32_t ByteNum)
{
	uint8_t addr,PageNum,DataSingle,count;
	addr=Addr%Flash_PageByteNum;
	PageNum=ByteNum/Flash_PageByteNum;
	DataSingle=ByteNum%Flash_PageByteNum;
	count=Flash_PageByteNum-addr;
	//如果地址对齐
	if(addr==0)
	{
		//输入数据不足一页
		if (PageNum==0)
		{
			SPI_Flash_Page_Program(Addr,WriteBuff,ByteNum);
			SPI_Flash_Waitfor();//还需要等待写入时序吗？？
		}
		//输入大于一页
		else
		{
			//数据正好填满每一页
			if(DataSingle==0)
			{
				while(PageNum--)
				{
					SPI_Flash_Page_Program(Addr,WriteBuff,Flash_PageByteNum);
		    	SPI_Flash_Waitfor();
					Addr=Addr+Flash_PageByteNum;
					WriteBuff=WriteBuff+Flash_PageByteNum;
				}
		 	}
			else
			{
				while(PageNum--)
				{
					SPI_Flash_Page_Program(Addr,WriteBuff,Flash_PageByteNum);
		    	SPI_Flash_Waitfor();
					Addr += Flash_PageByteNum;
					WriteBuff += Flash_PageByteNum;
				}
				SPI_Flash_Page_Program(Addr,WriteBuff,DataSingle);
				SPI_Flash_Waitfor();
			}
		}
	}
	//地址不对齐
	else
	{
		//如果输入数据数小于count
		if(ByteNum<count)
		{
			SPI_Flash_Page_Program(Addr,WriteBuff,ByteNum);
			SPI_Flash_Waitfor();//还需要等待写入时序吗？？
		}
		else
		{
			SPI_Flash_Page_Program(Addr,WriteBuff,count);
			SPI_Flash_Waitfor();
			ByteNum -= count;
		  PageNum=ByteNum/Flash_PageByteNum;
	    DataSingle=ByteNum%Flash_PageByteNum;
		  WriteBuff += count;
		  Addr += count;
			if (PageNum==0)
			{
			  SPI_Flash_Page_Program(Addr,WriteBuff,ByteNum);
			  SPI_Flash_Waitfor();//还需要等待写入时序吗？？
		  }
		//输入大于一页
		  else
		  {
			  //数据正好填满每一页
			  if(DataSingle==0)
			  {
				  while(PageNum--)
				  {
					  SPI_Flash_Page_Program(Addr,WriteBuff,Flash_PageByteNum);
		    	  SPI_Flash_Waitfor();
					  Addr=Addr+Flash_PageByteNum;
					  WriteBuff=WriteBuff+Flash_PageByteNum;
				  }
		 	  }
			  else
			  {
				  while(PageNum--)
				  {
					  SPI_Flash_Page_Program(Addr,WriteBuff,Flash_PageByteNum);
		    	  SPI_Flash_Waitfor();
					  Addr += Flash_PageByteNum;
					  WriteBuff += Flash_PageByteNum;
				  }
				  SPI_Flash_Page_Program(Addr,WriteBuff,DataSingle);
				  SPI_Flash_Waitfor();
			  }
		  }
	  }
  }
}


//等待内部时序结束
uint32_t SPI_Flash_Waitfor(void)
{

	GPIO_ResetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
	SPI_Flash_WritByte(Instruction_Read_Status_Register);
	while(SPI_Flash_WritByte(FLASH_Invali_Data)&0x01);
	GPIO_SetBits(FLASH_SPI_GPIO_CS_PORT, FLASH_SPI_GPIO_CS_PIN);
	return 0;
}

static  uint32_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* Block communication and all processes */
  EEPROM_ERROR("SPI 等待超时!errorCode = %d",errorCode);
  
  return 0;
}


