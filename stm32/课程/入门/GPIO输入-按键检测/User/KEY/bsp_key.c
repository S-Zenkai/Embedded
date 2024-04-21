#include "bsp_key.h"

void KEY1_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin=KEY1_GPIO_Pin;
	GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);
}

void KEY2_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin=KEY2_GPIO_Pin;
	GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);
}

uint8_t KEY_CHECK(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
	if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==1)
	{
	  while( GPIO_ReadInputDataBit(GPIOx, GPIO_Pin)==1);
		return 1;
	}
	else
		return 0;

}

