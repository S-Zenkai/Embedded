#include "stm32f10x.h"
#include "bsp.h"


void Delay(uint32_t count)
{
	for(;count > 0;count--)
		;
}
int main(void)
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	while(1)
	{
		LED_G_Config();
		LED_G(ON);
	  Delay(0xFFFFF);
	  LED_G(OFF);
	  Delay(0xFFFFF);
		
		LED_B_Config();
		LED_B(ON);
	  Delay(0xFFFFF);
		LED_B(OFF);
	  Delay(0xFFFFF);
		
		LED_R_Config();
		LED_R(ON);
	  Delay(0xFFFFF);
		LED_R(OFF);
	  Delay(0xFFFFF);
	}
}



