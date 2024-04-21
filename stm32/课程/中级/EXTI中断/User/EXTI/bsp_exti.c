#include "bsp_exti.h"
#include "bsp.h"

static void NVIC_EXTI_Config(void)
{
  //配置优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//KEY1_NVIC初始化
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	//KEY2_NVIC初始化
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStruct);
}


void KEY_EXTI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	
	//配置优先级（为什么）
	NVIC_EXTI_Config();
	//KEY1按键初始化
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK|KEY2_GPIO_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = KEY1_GPIO_Pin;
	GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);
	//KEY2按键初始化
	GPIO_InitStruct.GPIO_Pin = KEY2_GPIO_Pin;
	GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);
	//KEY1_AFIO配置（配置EXTI必须使能EXTI)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA , GPIO_PinSource0);
	//KEY2_AFIO配置
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC , GPIO_PinSource13);
	//KEY1_EXTI初始化
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	//KEY2_EXTI初始化
	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStruct);
}






