#include "bsp_SisTick.h"

//��ʱʱ����㣺T=us*(SystemCoreClock/100000)/72M
//
void Delay_SisTick_us(__IO uint32_t us)
{
	SysTick_Config(SystemCoreClock/100000);
	//��ÿ����ʱ���ڽ�����CTRL��λ16����1
	while(us != 0)
	{
		while((SysTick->CTRL & (1<<16)) == 0);
		us--;
	}
	//�رն�ʱ��
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}


void Delay_SisTick_ms(__IO uint32_t ms)
{
	SysTick_Config(SystemCoreClock/1000);
	//��ÿ����ʱ���ڽ�����CTRL��λ16����1
	while(ms != 0)
	{
		while((SysTick->CTRL & (1<<16)) == 0);
		ms--;
	}
	//�رն�ʱ��
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
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


