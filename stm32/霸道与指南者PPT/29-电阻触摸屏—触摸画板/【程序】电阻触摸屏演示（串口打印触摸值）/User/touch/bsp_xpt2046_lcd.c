/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   触摸屏驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 


#include "./touch/bsp_xpt2046_lcd.h"


 /**
  * @brief  触摸屏初始化
  * @param  无
  * @retval 无
  */
void XP2046_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 打开SPI GPIO的时钟
	RCC_APB2PeriphClockCmd(XPT2046_SPI_GPIO_CLK, ENABLE);	

	// 将CS CLK MOSI 的GPIO配置为推挽输出模式
	GPIO_InitStructure.GPIO_Pin = XPT2046_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(XPT2046_SPI_CS_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = XPT2046_SPI_CLK_PIN;
	GPIO_Init(XPT2046_SPI_CLK_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = XPT2046_SPI_MOSI_PIN;
	GPIO_Init(XPT2046_SPI_MOSI_PORT, &GPIO_InitStructure);		

  // 将PENIRQ MISO 的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = XPT2046_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(XPT2046_SPI_MISO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = XPT2046_PENIRQ_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(XPT2046_PENIRQ_GPIO_PORT, &GPIO_InitStructure);
		
}



/**
  * @brief  用于 XPT2046 的简单微秒级延时函数
  * @param  nCount ：延时计数值，单位为微妙
  * @retval 无
  */	
static void XPT2046_DelayUS ( __IO uint32_t ulCount )
{
	uint32_t i;


	for ( i = 0; i < ulCount; i ++ )
	{
		uint8_t uc = 12;     //设置值为12，大约延1微秒  
	      
		while ( uc -- );     //延1微秒	

	}
	
}


//发送命令
void XPT2046_SendCMD(uint8_t cmd)	
{
	uint8_t i;
	
	/*设置引脚的初始状态*/
	XPT2046_CS_ENABLE();
	XPT2046_CLK_LOW();
	XPT2046_MOSI_0();
	
	/*片选选中*/
	XPT2046_DelayUS(10);
	XPT2046_CS_DISABLE();
	
	/*产生8个时钟，发送数据*/
	for(i=0;i<8;i++)
	{
		if( (cmd & (0x80>>i)) == 0 )
		{
			XPT2046_MOSI_0();
		}
		else
		{
			XPT2046_MOSI_1();		
		}		
		
		XPT2046_CLK_HIGH();		
		XPT2046_DelayUS(5);
			
		XPT2046_CLK_LOW();
		XPT2046_DelayUS(5);	
	}	
	
//	/*片选取消*/
//	XPT2046_CS_ENABLE();
	
}





//读取数据
uint16_t XPT2046_ReceiveData(void)	
{
	uint8_t i;
	
	uint16_t receive_temp=0;
	
//	/*设置引脚的初始状态*/
//	XPT2046_CS_ENABLE();
//	XPT2046_CLK_LOW();
//	XPT2046_MOSI_0();
//	
//	/*片选选中*/
//	XPT2046_DelayUS(10);
//	XPT2046_CS_DISABLE();
	
		XPT2046_CLK_HIGH();		
		XPT2046_DelayUS(5);
		XPT2046_CLK_LOW();
		XPT2046_DelayUS(5);	
	
	/*产生12个时钟，读取数据*/
	for(i=0;i < 12;i++)
	{		
		receive_temp = receive_temp << 1;

		XPT2046_CLK_HIGH();		
		XPT2046_DelayUS(5);
		
		
		if( XPT2046_MISO() == Bit_SET)
		{
			receive_temp |= 0x01;
		}
		else
		{
			receive_temp |= 0x00;		
		}				

			
		XPT2046_CLK_LOW();
		XPT2046_DelayUS(5);	
		

	}	
	
	/*片选取消*/
	XPT2046_CS_ENABLE();
	
	return receive_temp;
	
}





//触摸检测状态机
uint8_t touch_detect(void)
{
	static Touch_State touch_state = XPT2046_STATE_RELEASE; 
	static uint8_t i;
	uint8_t result;
	
	switch(touch_state)
	{
		case XPT2046_STATE_RELEASE:
						if(XPT2046_PENIRQ_Read() == XPT2046_PENIRQ_ActiveLevel)
						{
								touch_state = XPT2046_STATE_WAITING;
								result = TOUCH_NOT_PRESSED;
						}
						else
						{							
								touch_state = XPT2046_STATE_RELEASE;
								result = TOUCH_NOT_PRESSED;
						}						
		break;
		
		case XPT2046_STATE_WAITING:
				if(XPT2046_PENIRQ_Read() == XPT2046_PENIRQ_ActiveLevel)
						{
							i++;
							
							if(i>10)
							{
								touch_state = XPT2046_STATE_PRESSED;
								result = TOUCH_PRESSED;
							}		
							else
							{
								touch_state = XPT2046_STATE_WAITING;
								result = TOUCH_NOT_PRESSED;							
							}										
								
						}
						else
						{					
								i = 0;
								touch_state = XPT2046_STATE_RELEASE;
								result = TOUCH_NOT_PRESSED;
						}				
		
			break;
		
		case XPT2046_STATE_PRESSED:			
						if(XPT2046_PENIRQ_Read() == XPT2046_PENIRQ_ActiveLevel)
						{
								touch_state = XPT2046_STATE_PRESSED;
								result = TOUCH_PRESSED;
						}
						else
						{							
								touch_state = XPT2046_STATE_RELEASE;
								result = TOUCH_NOT_PRESSED;
						}	
			
			break;
	
	}
	
	return result;
}






