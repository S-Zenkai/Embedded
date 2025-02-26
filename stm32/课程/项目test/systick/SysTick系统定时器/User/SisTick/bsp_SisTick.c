#include "bsp_SisTick.h"

//定时时间计算：T=us*(SystemCoreClock/100000)/72M
//
void Delay_SisTick_us(__IO uint32_t us)
{
	SysTick_Config(SystemCoreClock/100000);
	//在每个定时周期结束后，CTRL的位16会置1
	while(us != 0)
	{
		while((SysTick->CTRL & (1<<16)) == 0);
		us--;
	}
	//关闭定时器
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}


void Delay_SisTick_ms(__IO uint32_t ms)
{
	SysTick_Config(SystemCoreClock/1000);
	//在每个定时周期结束后，CTRL的位16会置1
	while(ms != 0)
	{
		while((SysTick->CTRL & (1<<16)) == 0);
		ms--;
	}
	//关闭定时器
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}


//为什么设置SisTick_Init？SysTick_Config只需配置一次，否则可能出错，故把Init单列出来
void SisTick_Init(void)
{
	//捕获错误
	if(SysTick_Config(SystemCoreClock/1000))
	{
		while(1);
	}
}


