 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SRAM 底层应用函数bsp 
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./sram/bsp_sram.h"


/**
  * @brief  初始化控制SRAM的IO
  * @param  无
  * @retval 无
  */
static void SRAM_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* 使能SRAM相关的GPIO时钟 */

                         /*地址信号线*/
  RCC_APB2PeriphClockCmd(FSMC_A0_GPIO_CLK | FSMC_A1_GPIO_CLK | FSMC_A2_GPIO_CLK | 
                         FSMC_A3_GPIO_CLK | FSMC_A4_GPIO_CLK | FSMC_A5_GPIO_CLK |
                         FSMC_A6_GPIO_CLK | FSMC_A7_GPIO_CLK | FSMC_A8_GPIO_CLK |
                         FSMC_A9_GPIO_CLK | FSMC_A10_GPIO_CLK| FSMC_A11_GPIO_CLK| 
												 FSMC_A12_GPIO_CLK| FSMC_A13_GPIO_CLK|FSMC_A14_GPIO_CLK|
												 FSMC_A15_GPIO_CLK|FSMC_A16_GPIO_CLK|FSMC_A17_GPIO_CLK|FSMC_A18_GPIO_CLK|
                         /*数据信号线*/
                         FSMC_D0_GPIO_CLK | FSMC_D1_GPIO_CLK | FSMC_D2_GPIO_CLK | 
                         FSMC_D3_GPIO_CLK | FSMC_D4_GPIO_CLK | FSMC_D5_GPIO_CLK |
                         FSMC_D6_GPIO_CLK | FSMC_D7_GPIO_CLK | FSMC_D8_GPIO_CLK |
                         FSMC_D9_GPIO_CLK | FSMC_D10_GPIO_CLK| FSMC_D11_GPIO_CLK|
                         FSMC_D12_GPIO_CLK| FSMC_D13_GPIO_CLK| FSMC_D14_GPIO_CLK|
                         FSMC_D15_GPIO_CLK|  
                         /*控制信号线*/
                         FSMC_CS_GPIO_CLK  | FSMC_WE_GPIO_CLK | FSMC_OE_GPIO_CLK |
                         FSMC_UDQM_GPIO_CLK|FSMC_LDQM_GPIO_CLK, ENABLE);
												 


	 /*-- GPIO 配置 -----------------------------------------------------*/

  /* 通用 GPIO 配置 */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;       //配置为复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     
  
  /*A地址信号线 针对引脚配置*/
  GPIO_InitStructure.GPIO_Pin = FSMC_A0_GPIO_PIN; 
  GPIO_Init(FSMC_A0_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A1_GPIO_PIN; 
  GPIO_Init(FSMC_A1_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A2_GPIO_PIN; 
  GPIO_Init(FSMC_A2_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A3_GPIO_PIN; 
  GPIO_Init(FSMC_A3_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A4_GPIO_PIN; 
  GPIO_Init(FSMC_A4_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A5_GPIO_PIN; 
  GPIO_Init(FSMC_A5_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A6_GPIO_PIN; 
  GPIO_Init(FSMC_A6_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A7_GPIO_PIN; 
  GPIO_Init(FSMC_A7_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A8_GPIO_PIN; 
  GPIO_Init(FSMC_A8_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A9_GPIO_PIN; 
  GPIO_Init(FSMC_A9_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A10_GPIO_PIN; 
  GPIO_Init(FSMC_A10_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A11_GPIO_PIN; 
  GPIO_Init(FSMC_A11_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = FSMC_A12_GPIO_PIN; 
  GPIO_Init(FSMC_A12_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A13_GPIO_PIN; 
  GPIO_Init(FSMC_A13_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A14_GPIO_PIN; 
  GPIO_Init(FSMC_A14_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A15_GPIO_PIN; 
  GPIO_Init(FSMC_A15_GPIO_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = FSMC_A16_GPIO_PIN; 
  GPIO_Init(FSMC_A16_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A17_GPIO_PIN; 
  GPIO_Init(FSMC_A17_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_A18_GPIO_PIN; 
  GPIO_Init(FSMC_A18_GPIO_PORT, &GPIO_InitStructure);
    
  /*DQ数据信号线 针对引脚配置*/
  GPIO_InitStructure.GPIO_Pin = FSMC_D0_GPIO_PIN; 
  GPIO_Init(FSMC_D0_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D1_GPIO_PIN; 
  GPIO_Init(FSMC_D1_GPIO_PORT, &GPIO_InitStructure);
    
  GPIO_InitStructure.GPIO_Pin = FSMC_D2_GPIO_PIN; 
  GPIO_Init(FSMC_D2_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D3_GPIO_PIN; 
  GPIO_Init(FSMC_D3_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D4_GPIO_PIN; 
  GPIO_Init(FSMC_D4_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D5_GPIO_PIN; 
  GPIO_Init(FSMC_D5_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D6_GPIO_PIN; 
  GPIO_Init(FSMC_D6_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D7_GPIO_PIN; 
  GPIO_Init(FSMC_D7_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D8_GPIO_PIN; 
  GPIO_Init(FSMC_D8_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D9_GPIO_PIN; 
  GPIO_Init(FSMC_D9_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D10_GPIO_PIN; 
  GPIO_Init(FSMC_D10_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D11_GPIO_PIN; 
  GPIO_Init(FSMC_D11_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D12_GPIO_PIN; 
  GPIO_Init(FSMC_D12_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D13_GPIO_PIN; 
  GPIO_Init(FSMC_D13_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D14_GPIO_PIN; 
  GPIO_Init(FSMC_D14_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_D15_GPIO_PIN; 
  GPIO_Init(FSMC_D15_GPIO_PORT, &GPIO_InitStructure);
  
  /*控制信号线*/
  GPIO_InitStructure.GPIO_Pin = FSMC_CS_GPIO_PIN; 
  GPIO_Init(FSMC_CS_GPIO_PORT, &GPIO_InitStructure);
    
  GPIO_InitStructure.GPIO_Pin = FSMC_WE_GPIO_PIN; 
  GPIO_Init(FSMC_WE_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_OE_GPIO_PIN; 
  GPIO_Init(FSMC_OE_GPIO_PORT, &GPIO_InitStructure);    
  
  GPIO_InitStructure.GPIO_Pin = FSMC_UDQM_GPIO_PIN; 
  GPIO_Init(FSMC_UDQM_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = FSMC_LDQM_GPIO_PIN; 
  GPIO_Init(FSMC_LDQM_GPIO_PORT, &GPIO_InitStructure);	
}		


//写时序的要求

//1.  ADDSET+1+DATAST+1  >55ns

//2.  DATAST+1  > 40ns

//3.  ADDSET+1 > 0ns

//读时序的要求

//1.  ADDSET+1+DATAST+1 +2 >55ns

//2.  DATAST+1  > 25ns

//3.  ADDSET+1 > 0ns    


//1 个HCLK时钟周期:
//T = 1/72MHz = 1.38*10^-8 s = 13.8 ns


//读时序的要求
//ADDSET = 0 
//DATAST =1 (经测试，DATAST配置成1是不能正常工作的)

//1.  ADDSET+1+DATAST+1 +2 >55ns  ------- 0+1+1+1+2 = 69 ns > 55ns

//2.  DATAST+1  > 25ns  ------ 1+1 = 27.6 >25 ns

//3.  ADDSET+1 > 0ns    ------  0+1 = 13.8ns >0 ns 



//写时序的要求
//ADDSET = 0 
//DATAST = 2


//1.  ADDSET+1+DATAST+1  >55ns  ------ 0+1+2+1 =55.2 >55ns 

//2.  DATAST+1  > 40ns ------ 2+1 = 41.4 >40 ns

//3.  ADDSET+1 > 0ns   ------  0+1 = 13.8ns >0 ns 

//初始化FSMC模式
static void FSMC_ModeConfig(void)
{
	FSMC_NORSRAMTimingInitTypeDef readTimInitStruct;
	FSMC_NORSRAMTimingInitTypeDef writeTimInitStruct;
	FSMC_NORSRAMInitTypeDef SRAMInitStruct;
	
	//使能FSMC时钟
	RCC_AHBPeriphClockCmd (RCC_AHBPeriph_FSMC ,ENABLE);
	
	//读
	readTimInitStruct.FSMC_AccessMode = FSMC_AccessMode_A;
	readTimInitStruct.FSMC_AddressHoldTime = 0;//SRAM没用到
	readTimInitStruct.FSMC_AddressSetupTime = 0; //理论值
	readTimInitStruct.FSMC_BusTurnAroundDuration =0;//SRAM没用到
	readTimInitStruct.FSMC_CLKDivision = 0;//SRAM没用到
	readTimInitStruct.FSMC_DataLatency = 0;//SRAM没用到
	readTimInitStruct.FSMC_DataSetupTime = 2; //2是经验值，理论值为1，后面再测试
		
	//写
	writeTimInitStruct.FSMC_AccessMode = FSMC_AccessMode_A;
	writeTimInitStruct.FSMC_AddressHoldTime = 0;//SRAM没用到
	writeTimInitStruct.FSMC_AddressSetupTime = 0; //理论值
	writeTimInitStruct.FSMC_BusTurnAroundDuration =0;//SRAM没用到
	writeTimInitStruct.FSMC_CLKDivision = 0;//SRAM没用到
	writeTimInitStruct.FSMC_DataLatency = 0;//SRAM没用到
	writeTimInitStruct.FSMC_DataSetupTime = 2; //2是理论值
	
	
	SRAMInitStruct.FSMC_Bank = FSMC_Bank1_NORSRAM3;
	SRAMInitStruct.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; 
	SRAMInitStruct.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	SRAMInitStruct.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	
	//SRAM没用到
	SRAMInitStruct.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable ;
	SRAMInitStruct.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable ;
	SRAMInitStruct.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable ;
	SRAMInitStruct.FSMC_WaitSignal =FSMC_WaitSignal_Disable ;
	SRAMInitStruct.FSMC_WaitSignalActive =FSMC_WaitSignalActive_BeforeWaitState  ;
	SRAMInitStruct.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low  ;
	SRAMInitStruct.FSMC_WrapMode = FSMC_WrapMode_Disable;
	SRAMInitStruct.FSMC_WriteBurst= FSMC_WriteBurst_Disable;
	SRAMInitStruct.FSMC_WriteOperation= FSMC_WriteOperation_Enable;
	
	//读写时序结构体
	SRAMInitStruct.FSMC_ReadWriteTimingStruct = &readTimInitStruct;
	SRAMInitStruct.FSMC_WriteTimingStruct =&writeTimInitStruct; //FSMC_ExtendedMode配置成Enable时有效
	
	//把配置写入到寄存器
	FSMC_NORSRAMInit(&SRAMInitStruct);
	//使能FSMC
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3,ENABLE);

}	


//SRAM初始化
void SRAM_Init(void)
{
	SRAM_GPIO_Config();
	FSMC_ModeConfig();
}


   
/*********************************************END OF FILE**********************/
