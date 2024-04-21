#include "bsp_SisTick.h"
void Delay_SisTick_us(uint32_t us)
{
	//uint32_t i=0;
	SysTick_Config(SystemCoreClock/100000);
	while(us != 0)
	{
		while((SysTick->CTRL & (1<<16)) == 0);
		us--;
	}
//	for(i=0;i<us;i++)
//	{
//		while((SysTick->CTRL & (1<<16)) == 0);
//	}
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;//这里要不要？
}

__IO uint32_t SisTick_Counter;
void Delay_SisTick_ms(__IO uint32_t ms)
{
	SysTick_Config(SystemCoreClock/1000);
	SisTick_Counter=ms;
	while(SisTick_Counter!=0);
}
