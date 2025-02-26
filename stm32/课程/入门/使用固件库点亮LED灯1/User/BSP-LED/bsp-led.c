#include "bsp-led.h"

void LED_G_Config(void)
{
	GPIO_InitTypeDef GPIOB_InitStruct;
	RCC_APB2PeriphClockCmd(LED_G_CLK, ENABLE);
	GPIOB_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIOB_InitStruct.GPIO_Pin=LED_G_Pin;
	GPIOB_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(LED_G_Port,&GPIOB_InitStruct);
}

void LED_B_Config(void)
{
	GPIO_InitTypeDef GPIOB_InitStruct;
	RCC_APB2PeriphClockCmd(LED_B_CLK, ENABLE);
	GPIOB_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIOB_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIOB_InitStruct.GPIO_Pin=LED_B_Pin;
	GPIO_Init(LED_B_Port,&GPIOB_InitStruct);
}

void LED_R_Config(void)
{
	GPIO_InitTypeDef GPIOB_InitStruct;
	RCC_APB2PeriphClockCmd(LED_B_CLK, ENABLE);
	GPIOB_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIOB_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIOB_InitStruct.GPIO_Pin=LED_R_Pin;
	GPIO_Init(LED_R_Port,&GPIOB_InitStruct);
}


