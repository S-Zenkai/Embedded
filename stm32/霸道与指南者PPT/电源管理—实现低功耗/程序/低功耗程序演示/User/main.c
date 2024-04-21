/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   按键测试（中断模式/EXTI模式）
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
#include "bsp_led.h"
#include "bsp_exti.h" 
#include "./usart/bsp_usart.h"


void Delay(__IO u32 nCount); 
static void SYSCLKConfig_STOP(void);


/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */ 
int main(void)
{
	/* LED 端口初始化 */
	LED_GPIO_Config();
	  	
	/* 初始化EXTI中断，按下按键会触发中断，
  *  触发中断会进入stm32f4xx_it.c文件中的函数
	*  KEY1_IRQHandler和KEY2_IRQHandler，处理中断，反转LED灯。
	*/
//	EXTI_Key_Config(); 
	
	USART_Config();
	
	
	LED_BLUE;
	
	printf("\r\n执行WFI语句之前");
	
	Delay(0xFFF);
	
//睡眠模式	
//	__WFI();	
	
//停止模式	
//	PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI); 
//	//恢复HSE时钟
//	SYSCLKConfig_STOP();
	
		
//待机模式	
//进入待机模式必须开启PWR时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	
	PWR_WakeUpPinCmd (ENABLE);
	PWR_EnterSTANDBYMode();
		
		
	printf("\r\n执行WFI语句之后");

	
	Delay(0xFFFFF);
	
	LED_RED;
	
	/* 等待中断，由于使用中断方式，CPU不用轮询按键 */
	while(1)                            
	{
		printf("\r\nwhile中的WFI语句之前");
		Delay(0xFFF);
		
//睡眠模式	
//	__WFI();	
	
//停止模式	
//	PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI); 
//	//恢复HSE时钟
//	SYSCLKConfig_STOP();
	
		


		printf("\r\nwhile中的WFI语句之后");

	}
}


void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}



/**
  * @brief  停机唤醒后配置系统时钟: 使能 HSE, PLL
  *         并且选择PLL作为系统时钟.
  * @param  None
  * @retval None
  */
static void SYSCLKConfig_STOP(void)
{
  /* After wake-up from STOP reconfigure the system clock */
  /* 使能 HSE */
  RCC_HSEConfig(RCC_HSE_ON);
  
  /* 等待 HSE 准备就绪 */
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
  {
  }
  
  /* 使能 PLL */ 
  RCC_PLLCmd(ENABLE);
  
  /* 等待 PLL 准备就绪 */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {
  }
  
  /* 选择PLL作为系统时钟源 */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  
  /* 等待PLL被选择为系统时钟源 */
  while (RCC_GetSYSCLKSource() != 0x08)
  {
  }
}
/*********************************************END OF FILE**********************/
