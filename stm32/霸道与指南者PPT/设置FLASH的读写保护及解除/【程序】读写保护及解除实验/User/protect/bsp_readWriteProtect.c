/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   读写保护驱动
	
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火STM32 F103-指南者 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "./protect/bsp_readWriteProtect.h"



void Write_Protect(void)
{

	FLASH_Unlock(); 
	
	FLASH_EraseOptionBytes();
	
	FLASH_EnableWriteProtection (FLASH_WRProt_AllPages );
	
	NVIC_SystemReset();

}


void Write_Protect_Disable(void)
{

	FLASH_Unlock(); 
	
	FLASH_EraseOptionBytes();
	
	FLASH_EnableWriteProtection (0x00000000 );
	
	NVIC_SystemReset();

}




void Read_Protect(void)
{

	FLASH_Unlock(); 
	
	FLASH_EraseOptionBytes();
	
	FLASH_ReadOutProtection (ENABLE );
	
//	NVIC_SystemReset();

}


void Read_Protect_Disable(void)
{

	FLASH_Unlock(); 
	
	FLASH_EraseOptionBytes();
	
	FLASH_ReadOutProtection (DISABLE );
	
//	NVIC_SystemReset();

}

	


