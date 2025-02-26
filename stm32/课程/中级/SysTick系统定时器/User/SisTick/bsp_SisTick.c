#include "bsp_SisTick.h"
//#include "misc.h"
#include "core_cm3.h"
static __IO uint32_t SisTick_Counter;
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
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;//����Ҫ��Ҫ��
}

//Ϊʲô����SisTick_Init��SysTick_Configֻ������һ�Σ�������ܳ����ʰ�Init���г���
void SisTick_Init(void)
{
	//�������
	if(SysTick_Config(SystemCoreClock/1000))
	{
		while(1);
	}
}

void Delay_SisTick_ms(__IO uint32_t ms)
{
	SisTick_Counter=ms;
	while(SisTick_Counter!=0);
}

void SisTick_Counter_Decrement(void)
{
	SisTick_Counter--;
}
