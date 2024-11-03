#include "bsp_clcconfig.h"


/**
  *@brief  时钟配置函数，利用HSE配置各个时钟，HSE为时钟源，通过PLL倍频为系统时钟
	*@param  pllmul，倍频因子
	*@retval 无
	*/
void HSE_SetSysClock(uint32_t pllmul)
{
	__IO uint32_t StartUpCounter = 0, HSEStartUpStatus = 0;
	//把RCC外设初始化成默认复位状态
	RCC_DeInit();
	//使能HSE
	RCC_HSEConfig(RCC_HSE_ON);
	//等待HSE启动
	HSEStartUpStatus=RCC_WaitForHSEStartUp();
	//HSE正常启动
	if(HSEStartUpStatus)
	{
		 // 使能FLASH 预存取缓冲区
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    // SYSCLK周期与闪存访问时间的比例设置，这里统一设置成2
		// 设置成2的时候，SYSCLK低于48M也可以工作，如果设置成0或者1的时候，
		// 如果配置的SYSCLK超出了范围的话，则会进入硬件错误，程序就死了
		// 0：0 < SYSCLK <= 24M
		// 1：24< SYSCLK <= 48M
		// 2：48< SYSCLK <= 72M
    FLASH_SetLatency(FLASH_Latency_2);
		//HCLK = SYSCLK
		//PCLK1=HCLK/2
		//PCLK2=HCLC
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//配置PLL时钟源、倍频因子
		//SYSCLK=PLLCLK*pllmul
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, pllmul);
		//使能PLL
		RCC_PLLCmd(ENABLE);
		/* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == SET)
    {
    }
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
		// 读取时钟切换状态位，确保PLLCLK被选为系统时钟
    while (RCC_GetSYSCLKSource() != 0x08)
    {
    }
	}
	else
	{
		/* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
	}
}


