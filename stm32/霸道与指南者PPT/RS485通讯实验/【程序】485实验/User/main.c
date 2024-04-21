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
  * 实验平台:秉火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "./485/bsp_485.h"
#include "./key/bsp_key.h"
#include "./led/bsp_led.h"   

//1.初始化485用的串口和 收发控制引脚
//2.编写中断服务函数


uint8_t sendData = 'A' ;


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  USART_Config();
	
	RS485_USART_Config();
	
	Key_GPIO_Config();
	LED_GPIO_Config();
	
	LED_RED;
	
	/* 发送一个字符串 */
	Usart_SendString( DEBUG_USARTx,"这是一个串口中断接收回显实验\n");
	printf("欢迎使用秉火STM32开发板\n\n\n\n");
	
  while(1)
	{	
		
		if(	Key_Scan(KEY1_GPIO_PORT,KEY1_GPIO_PIN) == KEY_ON)
		{
			
			printf("\r\n检测到按键按下，准备发送数据");
			
			RS485_Usart_SendArray(RS485_USARTx,&sendData,1);
		}
		
		

		
	}	
}
/*********************************************END OF FILE**********************/
