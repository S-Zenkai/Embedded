/**
 * *****************************************************************************
 * @file        bsp_btim.c
 * @brief       配置基本定时器tim6、tim7定时中断功能
 * @author      
 * @date        2024-11-17
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
 * 
 * *****************************************************************************
 */
#include "bsp_btim.h"
#include "bsp_usart.h"

/**
 * @brief       中断优先级配置，NVIC_PriorityGroupConfig只需调用一次
 *
 */
static void NVIC_Configure(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief       TIM时基配置
 * 
 */
static void TIM_TimeBaseConfigure(void)
{
	Timx1_APBxClock_FUN(Timx1_CLK, ENABLE);
	Timx2_APBxClock_FUN(Timx2_CLK, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 7200;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 10000;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(Timx1, &TIM_TimeBaseInitStructure);
	/*防止开启定时器时触发中断*/
	TIM_ClearFlag(Timx1, TIM_FLAG_Update);
	TIM_TimeBaseInit(Timx2, &TIM_TimeBaseInitStructure);
	TIM_ClearFlag(Timx2, TIM_FLAG_Update);
}

/**
 * @brief       TIM初始化
 * 
 */
void TIM_Init(void)
{
	TIM_TimeBaseConfigure();/*要先配置时基，放下面无法触发中断*/
	NVIC_Configure();
	TIM_ITConfig(Timx1, TIM_IT_Update, ENABLE);
	TIM_ITConfig(Timx2, TIM_IT_Update, ENABLE);
	TIM_Cmd(Timx1, ENABLE);
	TIM_Cmd(Timx2, ENABLE);
}

/**
 * @brief       
 * 
 */
void TIM6_IRQHandler(void)
{
	if (TIM_GetFlagStatus(Timx1, TIM_FLAG_Update))
	{
		printf("TIMx1中断\n");/*在中断函数中尽量不要调用printf,该函数运行时间较长，此处调用仅为了展现实验现象*/
		TIM_ClearFlag(Timx1, TIM_FLAG_Update);
	}
}

void TIM7_IRQHandler(void)
{
	if (TIM_GetFlagStatus(Timx2, TIM_FLAG_Update))
	{
		printf("TIMx2中断\n"); /*在中断函数中尽量不要调用printf,该函数运行时间较长，此处调用仅为了展现实验现象*/
		TIM_ClearFlag(Timx2, TIM_FLAG_Update);
	}
}
