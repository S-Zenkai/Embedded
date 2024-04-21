#include "stm32f10x.h"
#include "bsp.h"
#include "bsp_exti.h"

int main(void)
{
	LED_G_Config();
	LED_R_Config();
	KEY_EXTI_Config();
//	KEY2_EXTI_Config();
	while(1);
}



