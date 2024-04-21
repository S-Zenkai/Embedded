/**
  ******************************************************************************
  * @file    bsp_i2c_ee.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SPI-FLASH驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./flash/bsp_spi_flash.h"
#include "./usart/bsp_usart.h"		



static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;     


static uint32_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);


/**
  * @brief  SPII/O配置
  * @param  无
  * @retval 无
  */
static void SPI_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 

	/* 使能与SPI 有关的时钟 */
	FLASH_SPI_APBxClock_FUN ( FLASH_SPI_CLK, ENABLE );
	FLASH_SPI_GPIO_APBxClock_FUN ( FLASH_SPI_GPIO_CLK, ENABLE );
	
    
  /* MISO MOSI SCK*/
  GPIO_InitStructure.GPIO_Pin = FLASH_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       
  GPIO_Init(FLASH_SPI_SCK_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = FLASH_SPI_MOSI_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       
  GPIO_Init(FLASH_SPI_MOSI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = FLASH_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	       
  GPIO_Init(FLASH_SPI_MISO_PORT, &GPIO_InitStructure);
	
	//初始化CS引脚，使用软件控制，所以直接设置成推挽输出	
	GPIO_InitStructure.GPIO_Pin = FLASH_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	       
  GPIO_Init(FLASH_SPI_CS_PORT, &GPIO_InitStructure);
	
	FLASH_SPI_CS_HIGH;
}


/**
  * @brief  SPI 工作模式配置
  * @param  无
  * @retval 无
  */
static void SPI_Mode_Config(void)
{
  SPI_InitTypeDef  SPI_InitStructure; 

	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2 ;
	//SPI 使用模式3
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge ;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High ;
	SPI_InitStructure.SPI_CRCPolynomial = 0;//不使用CRC功能，数值随便写
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex ;//双线全双工
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB  ;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master  ;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft  ;	
	
	SPI_Init(FLASH_SPIx,&SPI_InitStructure);	//写入配置到寄存器
	
	SPI_Cmd(FLASH_SPIx,ENABLE);//使能SPI
	
}


/**
  * @brief  SPI 初始化
  * @param  无
  * @retval 无
  */
void SPI_FLASH_Init(void)
{
	
	SPI_GPIO_Config();
	SPI_Mode_Config();

}

//发送并接收一个字节
uint8_t SPI_FLASH_Send_Byte(uint8_t data)
{
	SPITimeout = SPIT_FLAG_TIMEOUT;
	//检查并等待至TX缓冲区为空
	while(SPI_I2S_GetFlagStatus(FLASH_SPIx,SPI_I2S_FLAG_TXE) == RESET)
	{
		if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
	}
	
	//程序执行到此处，TX缓冲区已空
	SPI_I2S_SendData (FLASH_SPIx,data);
	
	
	SPITimeout = SPIT_FLAG_TIMEOUT;
	//检查并等待至RX缓冲区为非空
	while(SPI_I2S_GetFlagStatus(FLASH_SPIx,SPI_I2S_FLAG_RXNE) == RESET)
	{
		if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
	}
		
	//程序执行到此处，说明数据发送完毕，并接收到一字字节	
	return SPI_I2S_ReceiveData(FLASH_SPIx); 

}

uint8_t SPI_FLASH_Read_Byte(void)
{
	return SPI_FLASH_Send_Byte(DUMMY); 
}



//读取ID号
uint32_t SPI_Read_ID(void)
{
	uint32_t flash_id;
	
	//片选使能
	FLASH_SPI_CS_LOW;
	SPI_FLASH_Send_Byte(READ_JEDEC_ID);
	
	flash_id = SPI_FLASH_Send_Byte(DUMMY);

	flash_id <<= 8;
	
	flash_id |= SPI_FLASH_Send_Byte(DUMMY); 
	
	flash_id <<= 8;
	
	flash_id |= SPI_FLASH_Send_Byte(DUMMY); 
	
	FLASH_SPI_CS_HIGH;	
	
	return flash_id;
}

//FLASH写入使能
void SPI_Write_Enable(void)
{
		//片选使能
	FLASH_SPI_CS_LOW;
	SPI_FLASH_Send_Byte(WRITE_ENABLE);	 
	FLASH_SPI_CS_HIGH;	
}



//擦除FLASH指定扇区
void SPI_Erase_Sector(uint32_t addr)
{	
	SPI_Write_Enable();
		//片选使能
	FLASH_SPI_CS_LOW;
	SPI_FLASH_Send_Byte(ERASE_SECTOR);
	
	SPI_FLASH_Send_Byte((addr>>16)&0xff);
	
	SPI_FLASH_Send_Byte((addr>>8)&0xff); 
	
  SPI_FLASH_Send_Byte(addr&0xff); 
	
	FLASH_SPI_CS_HIGH;	
	
	SPI_WaitForWriteEnd();
	
}


//读取FLASH的内容
void SPI_Read_Data(uint32_t addr,uint8_t *readBuff,uint32_t numByteToRead)
{
		//片选使能
	FLASH_SPI_CS_LOW;
	SPI_FLASH_Send_Byte(READ_DATA);
	
	SPI_FLASH_Send_Byte((addr>>16)&0xff);
	
	SPI_FLASH_Send_Byte((addr>>8)&0xff); 
	
  SPI_FLASH_Send_Byte(addr&0xff); 
	
	while(numByteToRead--)
	{	
		*readBuff = SPI_FLASH_Send_Byte(DUMMY);
		readBuff++;
	}
	
	
	FLASH_SPI_CS_HIGH;	
	
}




//向FLASH写入内容
//读取FLASH的内容
void SPI_Write_Data(uint32_t addr,uint8_t *writeBuff,uint32_t numByteToWrite)
{
	SPI_Write_Enable();
		//片选使能
	FLASH_SPI_CS_LOW;
	SPI_FLASH_Send_Byte(WRITE_DATA);
	
	SPI_FLASH_Send_Byte((addr>>16)&0xff);
	
	SPI_FLASH_Send_Byte((addr>>8)&0xff); 
	
  SPI_FLASH_Send_Byte(addr&0xff); 
	
	while(numByteToWrite--)
	{	
		SPI_FLASH_Send_Byte(*writeBuff);
		writeBuff++;
	}
	
	
	FLASH_SPI_CS_HIGH;	
	SPI_WaitForWriteEnd();
}



//等待FLASH内部时序操作完成
void SPI_WaitForWriteEnd(void)
{
	uint8_t status_reg = 0;
	
	//片选使能
	FLASH_SPI_CS_LOW;
	
	SPI_FLASH_Send_Byte(READ_STATUS);
	
	do
	{	
	status_reg = SPI_FLASH_Send_Byte(DUMMY);
	}
	while((status_reg & 0x01) == 1);
	
	FLASH_SPI_CS_HIGH;	


}



/**
  * @brief  Basic management of the timeout situation.
  * @param  errorCode：错误代码，可以用来定位是哪个环节出错.
  * @retval 返回0，表示SPI读取失败.
  */
static  uint32_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* Block communication and all processes */
  FLASH_ERROR("SPI 等待超时!errorCode = %d",errorCode);
  
  return 0;
}
/*********************************************END OF FILE**********************/

