/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "./key/bsp_key.h"  
#include "./protect/bsp_readWriteProtect.h"

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  USART_Config();
	
	Key_GPIO_Config();
	
	/* 发送一个字符串 */
	printf("欢迎使用秉火STM32开发板\n\n\n\n");
	
//	printf("写保护状态：0x%08x\n\n\n\n",FLASH_GetWriteProtectionOptionByte());
	
	
	printf("读保护状态：0x%x\n\n\n\n",FLASH_GetReadOutProtectionStatus());

	
  while(1)
	{	
		
		if( Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON  )
		{
			printf("即将实施 写/读 保护\n\n\n\n");
			
//			Write_Protect();

			Read_Protect();
		
		
		}
		
		if( Key_Scan(KEY2_GPIO_PORT,KEY2_GPIO_PIN) == KEY_ON  )
		{
			printf("即将解除 写/读 保护\n\n\n\n");
			
//			Write_Protect_Disable();

			Read_Protect_Disable();
		
		}

		
		
	}	
}
/*********************************************END OF FILE**********************/
