#include "bsp_led.h"

void LED_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin=LED_G_GPIO_Pin;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(LED_G_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=LED_R_GPIO_Pin;
	GPIO_Init(LED_R_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=LED_B_GPIO_Pin;
	GPIO_Init(LED_B_GPIO_PORT, &GPIO_InitStruct);
	LED_OFF;
}



	





