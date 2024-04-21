
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_AdvanceTim.h"

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  //USART_Config();
	
  /* 高级定时器初始化 */
	ADVANCE_TIM_Init();
	
  while(1)
	{	
		
	}	
}

// 1- 把PWM信号的频率改成100KHZ

// 2- 使用通用定时器输出4路占空比不同的PWM信号

/*********************************************END OF FILE**********************/
