 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief  LCD 驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./lcd/bsp_ili9341_lcd.h"





void ILI9341_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 打开GPIO的时钟
	RCC_APB2PeriphClockCmd(ILI9341_CS_CLK|ILI9341_RST_CLK|ILI9341_BK_CLK|ILI9341_RD_CLK|
						ILI9341_WR_CLK|ILI9341_DC_CLK|ILI9341_D0_CLK|ILI9341_D1_CLK|ILI9341_D2_CLK|
						ILI9341_D3_CLK|ILI9341_D4_CLK|ILI9341_D5_CLK|ILI9341_D6_CLK|ILI9341_D7_CLK|
						ILI9341_D8_CLK|ILI9341_D9_CLK|ILI9341_D10_CLK|ILI9341_D11_CLK|ILI9341_D12_CLK|
						ILI9341_D13_CLK|ILI9341_D14_CLK|ILI9341_D15_CLK	, ENABLE);
	
	// 将的GPIO配置为推挽模式
	GPIO_InitStructure.GPIO_Pin = ILI9341_BK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ILI9341_BK_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_RST_PIN;
	GPIO_Init(ILI9341_RST_PORT, &GPIO_InitStructure);
	
	
	// 将的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = ILI9341_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ILI9341_CS_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_RD_PIN;
	GPIO_Init(ILI9341_RD_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_WR_PIN;
	GPIO_Init(ILI9341_WR_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_DC_PIN;
	GPIO_Init(ILI9341_DC_PORT, &GPIO_InitStructure);
	
	
		/* 配置FSMC相对应的数据线,FSMC-D0~D15 */	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D0_PIN;
	GPIO_Init ( ILI9341_D0_PORT, & GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = ILI9341_D1_PIN;
	GPIO_Init ( ILI9341_D1_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D2_PIN;
	GPIO_Init ( ILI9341_D2_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D3_PIN;
	GPIO_Init ( ILI9341_D3_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D4_PIN;
	GPIO_Init ( ILI9341_D4_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D5_PIN;
	GPIO_Init ( ILI9341_D5_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D6_PIN;
	GPIO_Init ( ILI9341_D6_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D7_PIN;
	GPIO_Init ( ILI9341_D7_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D8_PIN;
	GPIO_Init ( ILI9341_D8_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D9_PIN;
	GPIO_Init ( ILI9341_D9_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D10_PIN;
	GPIO_Init ( ILI9341_D10_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D11_PIN;
	GPIO_Init ( ILI9341_D11_PORT, & GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = ILI9341_D12_PIN;
	GPIO_Init ( ILI9341_D12_PORT, & GPIO_InitStructure );	
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D13_PIN;
	GPIO_Init ( ILI9341_D13_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D14_PIN;
	GPIO_Init ( ILI9341_D14_PORT, & GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = ILI9341_D15_PIN;
	GPIO_Init ( ILI9341_D15_PORT, & GPIO_InitStructure );


}



void ILI9341_FSMC_Config(void)
{
	
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming;

		
	/*使能FSMC外设时钟*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC,ENABLE);

	//地址建立时间（ADDSET）
	readWriteTiming.FSMC_AddressSetupTime = 0x01;	
	//数据保持时间（DATAST）
	readWriteTiming.FSMC_DataSetupTime = 0x04;		 

	
	//地址保持时间（ADDHLD）模式B未用到
	readWriteTiming.FSMC_AddressHoldTime = 0x00;	 
	
	//设置总线转换周期，仅用于复用模式的NOR操作
	readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
	
	//设置时钟分频，仅用于同步类型的存储器
	readWriteTiming.FSMC_CLKDivision = 0x00;	

	//数据保持时间，仅用于NOR
	readWriteTiming.FSMC_DataLatency = 0x00;		
	
	//选择匹配SRAM的模式
	readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_B;	 
    

	// 选择FSMC映射的存储区域： Bank1 NORSRAMx
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_BANK_NORSRAMx; 
	
	//设置地址总线与数据总线是否复用，仅用于NOR
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; 
	
	//设置要控制的存储器类型：NOR类型
	FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_NOR;   
	
	//存储器数据宽度：16位
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b; 
	
	//设置是否使用突发访问模式，仅用于同步类型的存储器
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;
	
	//设置是否使能等待信号，仅用于同步类型的存储器
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable;
	
	//设置等待信号的有效极性，仅用于同步类型的存储器
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	
	//设置是否支持把非对齐的突发操作，仅用于同步类型的存储器
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable; 
	
	//设置等待信号插入的时间，仅用于同步类型的存储器
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	
	//存储器写使能 
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	
	//不使用等待信号
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;  		
	
	// 不使用扩展模式，读写使用相同的时序
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable; 
	
	//突发写操作
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;  
	
	//读写时序配置
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
	
	//读写同样时序，使用扩展模式时这个配置才有效
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &readWriteTiming; 

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //初始化FSMC配置

	FSMC_NORSRAMCmd(FSMC_BANK_NORSRAMx, ENABLE);  // 使能BANK										  
					
}



/**
  * @brief  向ILI9341写入命令
  * @param  usCmd :要写入的命令（表寄存器地址）
  * @retval 无
  */	
__inline void ILI9341_Write_Cmd ( uint16_t usCmd )
{
	 *ILI9341_CMD_ADDR = usCmd;	
}


/**
  * @brief  向ILI9341写入数据
  * @param  usData :要写入的数据
  * @retval 无
  */	
__inline void ILI9341_Write_Data ( uint16_t usData )
{
	* ILI9341_DATA_ADDR = usData;
	
}


/**
  * @brief  从ILI9341读取数据
  * @param  无
  * @retval 读取到的数据
  */	
__inline uint16_t ILI9341_Read_Data ( void )
{
	return ( *ILI9341_DATA_ADDR );
	
}

uint16_t Read_Pixel_Format(void)
{

	ILI9341_Write_Cmd(0x0C);
	ILI9341_Read_Data();
	
	return ILI9341_Read_Data();

}


/**
 * @brief  ILI9341背光LED控制
 * @param  enumState ：决定是否使能背光LED
  *   该参数为以下值之一：
  *     @arg ENABLE :使能背光LED
  *     @arg DISABLE :禁用背光LED
 * @retval 无
 */
void ILI9341_BackLed_Control ( FunctionalState enumState )
{
	if ( enumState )
		GPIO_ResetBits ( ILI9341_BK_PORT, ILI9341_BK_PIN );	
	else
		GPIO_SetBits ( ILI9341_BK_PORT, ILI9341_BK_PIN );
		
}


/**
  * @brief  用于 ILI9341 简单延时函数
  * @param  nCount ：延时计数值
  * @retval 无
  */	
static void ILI9341_Delay ( __IO uint32_t nCount )
{
  for ( ; nCount != 0; nCount -- );
	
}


/**
 * @brief  ILI9341 软件复位
 * @param  无
 * @retval 无
 */
void ILI9341_Rst ( void )
{			
	GPIO_ResetBits ( ILI9341_RST_PORT, ILI9341_RST_PIN );	 //低电平复位

	ILI9341_Delay ( 0xAFF ); 					   

	GPIO_SetBits ( ILI9341_RST_PORT, ILI9341_RST_PIN );		 	 

	ILI9341_Delay ( 0xAFF ); 	
	
}


/**
 * @brief  初始化ILI9341寄存器
 * @param  无
 * @retval 无
 */
static void ILI9341_REG_Config ( void )
{
	/*  Power control B (CFh)  */
	DEBUG_DELAY  ();
	ILI9341_Write_Cmd ( 0xCF  );
	ILI9341_Write_Data ( 0x00  );
	ILI9341_Write_Data ( 0x81  );
	ILI9341_Write_Data ( 0x30  );
	
	/*  Power on sequence control (EDh) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xED );
	ILI9341_Write_Data ( 0x64 );
	ILI9341_Write_Data ( 0x03 );
	ILI9341_Write_Data ( 0x12 );
	ILI9341_Write_Data ( 0x81 );
	
	/*  Driver timing control A (E8h) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xE8 );
	ILI9341_Write_Data ( 0x85 );
	ILI9341_Write_Data ( 0x10 );
	ILI9341_Write_Data ( 0x78 );
	
	/*  Power control A (CBh) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xCB );
	ILI9341_Write_Data ( 0x39 );
	ILI9341_Write_Data ( 0x2C );
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x34 );
	ILI9341_Write_Data ( 0x02 );
	
	/* Pump ratio control (F7h) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xF7 );
	ILI9341_Write_Data ( 0x20 );
	
	/* Driver timing control B */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xEA );
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x00 );
	
	/* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xB1 );
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x1B );
	
	/*  Display Function Control (B6h) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xB6 );
	ILI9341_Write_Data ( 0x0A );
	ILI9341_Write_Data ( 0xA2 );
	
	/* Power Control 1 (C0h) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xC0 );
	ILI9341_Write_Data ( 0x35 );
	
	/* Power Control 2 (C1h) */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0xC1 );
	ILI9341_Write_Data ( 0x11 );
	
	/* VCOM Control 1 (C5h) */
	ILI9341_Write_Cmd ( 0xC5 );
	ILI9341_Write_Data ( 0x45 );
	ILI9341_Write_Data ( 0x45 );
	
	/*  VCOM Control 2 (C7h)  */
	ILI9341_Write_Cmd ( 0xC7 );
	ILI9341_Write_Data ( 0xA2 );
	
	/* Enable 3G (F2h) */
	ILI9341_Write_Cmd ( 0xF2 );
	ILI9341_Write_Data ( 0x00 );
	
	/* Gamma Set (26h) */
	ILI9341_Write_Cmd ( 0x26 );
	ILI9341_Write_Data ( 0x01 );
	DEBUG_DELAY ();
	
	/* Positive Gamma Correction */
	ILI9341_Write_Cmd ( 0xE0 ); //Set Gamma
	ILI9341_Write_Data ( 0x0F );
	ILI9341_Write_Data ( 0x26 );
	ILI9341_Write_Data ( 0x24 );
	ILI9341_Write_Data ( 0x0B );
	ILI9341_Write_Data ( 0x0E );
	ILI9341_Write_Data ( 0x09 );
	ILI9341_Write_Data ( 0x54 );
	ILI9341_Write_Data ( 0xA8 );
	ILI9341_Write_Data ( 0x46 );
	ILI9341_Write_Data ( 0x0C );
	ILI9341_Write_Data ( 0x17 );
	ILI9341_Write_Data ( 0x09 );
	ILI9341_Write_Data ( 0x0F );
	ILI9341_Write_Data ( 0x07 );
	ILI9341_Write_Data ( 0x00 );
	
	/* Negative Gamma Correction (E1h) */
	ILI9341_Write_Cmd ( 0XE1 ); //Set Gamma
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x19 );
	ILI9341_Write_Data ( 0x1B );
	ILI9341_Write_Data ( 0x04 );
	ILI9341_Write_Data ( 0x10 );
	ILI9341_Write_Data ( 0x07 );
	ILI9341_Write_Data ( 0x2A );
	ILI9341_Write_Data ( 0x47 );
	ILI9341_Write_Data ( 0x39 );
	ILI9341_Write_Data ( 0x03 );
	ILI9341_Write_Data ( 0x06 );
	ILI9341_Write_Data ( 0x06 );
	ILI9341_Write_Data ( 0x30 );
	ILI9341_Write_Data ( 0x38 );
	ILI9341_Write_Data ( 0x0F );
	
	/* memory access control set */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0x36 ); 	
	ILI9341_Write_Data ( 0xC8 );    /*竖屏  左上角到 (起点)到右下角 (终点)扫描方式*/
	DEBUG_DELAY ();
	
	/* column address control set */
	ILI9341_Write_Cmd ( CMD_SetCoordinateX );  
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0xEF );
	
	/* page address control set */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( CMD_SetCoordinateY ); 
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x00 );
	ILI9341_Write_Data ( 0x01 );
	ILI9341_Write_Data ( 0x3F );
	
	/*  Pixel Format Set (3Ah)  */
	DEBUG_DELAY ();
	ILI9341_Write_Cmd ( 0x3a ); 
	ILI9341_Write_Data ( 0x55 );
	
	/* Sleep Out (11h)  */
	ILI9341_Write_Cmd ( 0x11 );	
	ILI9341_Delay ( 0xAFFf<<2 );
	DEBUG_DELAY ();
	
	/* Display ON (29h) */
	ILI9341_Write_Cmd ( 0x29 ); 
	
	
}


void ILI9341_Init(void)
{
	ILI9341_GPIO_Config();
	ILI9341_FSMC_Config();
	
	ILI9341_BackLed_Control(ENABLE);
	ILI9341_Rst();
	ILI9341_REG_Config();
}

void ILI9341_Open_Window(uint16_t usX,uint16_t usY,uint16_t usWidth,uint16_t usHeight)
{
	/* column address control set */
	ILI9341_Write_Cmd ( CMD_SetCoordinateX );  
	ILI9341_Write_Data ( (usX&0xFF00) >> 8 );
	ILI9341_Write_Data ( (usX&0x00FF) );
	ILI9341_Write_Data ( ((usX+usWidth)&0xFF00) >> 8 );
	ILI9341_Write_Data ( ((usX+usWidth)&0x00FF) );	

	ILI9341_Write_Cmd ( CMD_SetCoordinateY ); 
	ILI9341_Write_Data ( (usY&0xFF00) >> 8 );
	ILI9341_Write_Data ( (usY&0x00FF) );
	ILI9341_Write_Data ( ((usY+usHeight)&0xFF00) >> 8 );
	ILI9341_Write_Data ( ((usY+usHeight)&0x00FF) );	
}

void ILI9341_Draw_Rec(void)
{
	uint32_t i;
	ILI9341_Open_Window(10,20,100,100);
	
	ILI9341_Write_Cmd(CMD_SetPixel);
	

	for(i=0;i<100*100;i++)
		ILI9341_Write_Data(RGB888_2_RGB565(79,67,188)); 
}

   
/*********************************************END OF FILE**********************/
