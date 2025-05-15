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
  * 实验平台:野火 F103-指南者 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
	int16_t a=65;
	uint8_t i=0;
		unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  USART_Config();
	
//	int16_t b =bigEndianToLittleEndian_int16(a);
	printf("%04d",a);
//			printf("%s\n",tail);
			for(i=0;i<4;i++)
			{
				printf("%c",tail[i]);
			}
	
  while(1)
	{	
		
	}	
}
/*********************************************END OF FILE**********************/
