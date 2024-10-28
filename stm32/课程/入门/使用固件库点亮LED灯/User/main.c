#include "stm32f10x.h"
#include "bsp_led.h"


void Delay(uint32_t count)
{
	for(;count > 0;count--)
		;
}
int main(void)
{
	LED_Config();
	while(1)
	{
		LED_R_ON;
		Delay(0xFFFFF);
		LED_R_OFF;
		Delay(0xFFFFF);
	  LED_G_ON;
		Delay(0xFFFFFF);
		LED_G_OFF;
		Delay(0xFFFFFF);
		LED_B_ON;
		Delay(0xFFFFFF);
		LED_B_OFF;
		Delay(0xFFFFFF);
		LED_Y_ON;
		Delay(0xFFFFFF);
		LED_Y_OFF;
		Delay(0xFFFFFF);
	}
}


