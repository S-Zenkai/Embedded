#include "stm32f10x.h"   // 相当于51单片机中的  #include <reg51.h>
#include "bsp_led.h"
#include "bsp_rccclkconfig.h"


void Delay( uint32_t count )
{
	for(; count!=0; count--);
}

int main(void)
{
	// 来到这里的时候，系统的时钟已经被配置成72M。
	
	//HSE_SetSysClk( RCC_PLLMul_9 );
	//HSI_SetSysClk( RCC_PLLMul_16 );
	
	MCO_GPIO_Config();
	RCC_MCOConfig(RCC_MCO_SYSCLK);
	//RCC_MCOConfig(RCC_MCO_HSI);
	
	LED_GPIO_Config();
	
	while(1)
	{
		
		LED_G(OFF);
		Delay(0xFFFFF);
		
		LED_G(ON);
		Delay(0xFFFFF);
	}
}


