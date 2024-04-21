#include "stm32f10x.h"
#include "bsp.h"
#include "bsp_SisTick.h"

int main(void)
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	while(1)
	{
		LED_B_Config();
		LED_B(ON);
	  Delay_SisTick_ms(1000);
	  LED_B(OFF);
	  Delay_SisTick_ms(1000);
	}
}



