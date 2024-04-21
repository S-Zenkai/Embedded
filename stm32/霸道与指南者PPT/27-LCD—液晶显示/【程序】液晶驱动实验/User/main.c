 /**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   LCD显示实验
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
#include "./usart/bsp_usart.h"
#include "./led/bsp_led.h"
#include "./lcd/bsp_ili9341_lcd.h"


//#define ILI9341_CMD_ADDR  		( uint16_t*)(0x60000000)
//#define ILI9341_DATA_ADDR  	( uint16_t*)(0x60020000)

extern uint16_t Read_Pixel_Format(void);

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
int main(void)
{ 	
	
//	uint32_t temp = 0x6C000000;
//	uint16_t lcd_read_temp;
	
	ILI9341_Init();
	
	LED_GPIO_Config();
	LED_BLUE;
	
	/* 配置串口为：115200 8-N-1 */
	USART_Config();
	printf("\r\n 这是一个LCD实验 \r\n");
	
	
	printf("\r\n0x0C命令返回值测试：0x%x",Read_Pixel_Format());
	ILI9341_Draw_Rec();
	
//	//向LCD发送命令
//	*ILI9341_CMD_ADDR = 0xABCD;
//	*( uint16_t*)(0x60000000) = 0xABCD;
//	
//	//向LCD发送数据
//	*ILI9341_DATA_ADDR = 0x1234;
//	*( uint16_t*)(0x60020000) = 0xABCD;
//	
//	//从液晶屏读取数据
//	lcd_read_temp = *ILI9341_DATA_ADDR;
//	lcd_read_temp = *( uint16_t*)(0x60020000);
//	
//	
//	
//	temp =temp | (1<<(23+1));	
//	
//	printf("\r\nA23为高电平，地址=0x%x",temp);
//	
//	temp = 0x6C000000;
//	
//	temp &=  ~(1<<(23+1));
//	
//	printf("\r\nA23为低电平，地址=0x%x",temp);

	while(1);  
}


void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
