#include "bsp_key.h"

void KEY1_Config(void)
{
	GPIO_InitTypeDef GPIOB_InitStruct;
	RCC_APB2PeriphClockCmd(KEY1_CLK, ENABLE);
	GPIOB_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIOB_InitStruct.GPIO_Pin=KEY1_Pin;
	GPIO_Init(KEY1_Port,&GPIOB_InitStruct);
}

void KEY2_Config(void)
{
	GPIO_InitTypeDef GPIOB_InitStruct;
	RCC_APB2PeriphClockCmd(KEY2_CLK, ENABLE);
	GPIOB_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIOB_InitStruct.GPIO_Pin=KEY2_Pin;
	GPIO_Init(KEY2_Port,&GPIOB_InitStruct);
}


