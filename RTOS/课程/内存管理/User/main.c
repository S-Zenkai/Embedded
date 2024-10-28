#include "stm32f10x.h"
#include <stdio.h>
#include "./KEY/bsp_key.h"
#include "./MM/bsp_mm.h"
#include "./USART/bsp_usart.h"
#include "./LED/bsp_led.h"



int main(void)
{
	KEY_Init();
	My_Mallco_Init();
	USART_Config();
	LED_Init();
	while(1)
	{
		if(Get_Keyx_Status(0)==1)
		{
			LEDG_TOGGLE;
		}
		if(Get_Keyx_Status(0)==2)
		{
			LEDB_TOGGLE;
		}
	}
}



