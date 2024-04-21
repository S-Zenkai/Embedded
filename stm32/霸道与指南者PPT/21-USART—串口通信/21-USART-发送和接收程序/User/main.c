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
  uint8_t a[10]={100,2,3,4,5,6,7,8,9,10};
	USART_Config();
	
//	Usart_SendByte(DEBUG_USARTx,'A');
//	Usart_SendHalfWord(DEBUG_USARTx, 0xff56);
//	
//	Usart_SendArray(DEBUG_USARTx, a,10);
//	Usart_SendStr(DEBUG_USARTx, "欢迎使用秉火STM32F103开发板 \n");
	
	printf( "串口printf函数测试\n" );
	
	while (1)
	{
	
	}
}
/*********************************************END OF FILE**********************/












