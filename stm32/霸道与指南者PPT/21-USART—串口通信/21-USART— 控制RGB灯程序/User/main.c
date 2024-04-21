/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   测试led
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
#include "bsp_led.h"
#include "bsp_usart.h"



/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{	
	uint8_t ch;
	USART_Config();
	LED_GPIO_Config();
	
	printf( "这是一个串口控制RGB灯的程序\n" );
 
	while (1)
	{
		ch = getchar();
	  printf( "ch=%c\n",ch );
		
		switch(ch)
   {
			case '1': LED_RED;
				break;
		 
			case '2': LED_GREEN;
			  break;
		 
			case '3': LED_BLUE;
			  break;
		 
			default: LED_RGBOFF;
		    break;
	 }
	}
}
/*********************************************END OF FILE**********************/












