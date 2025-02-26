/**
 * *****************************************************************************
 * @file        bsp_btim.c
 * @brief       ���û�����ʱ��tim6��tim7��ʱ�жϹ���
 * @author      
 * @date        2024-11-17
 * @version     0.1
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * ʵ��ƽ̨:
 * 
 * *****************************************************************************
 */
#include "bsp_btim.h"
#include "bsp_usart.h"

/**
 * @brief       �ж����ȼ����ã�NVIC_PriorityGroupConfigֻ�����һ��
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
 * @brief       TIMʱ������
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
	/*��ֹ������ʱ��ʱ�����ж�*/
	TIM_ClearFlag(Timx1, TIM_FLAG_Update);
	TIM_TimeBaseInit(Timx2, &TIM_TimeBaseInitStructure);
	TIM_ClearFlag(Timx2, TIM_FLAG_Update);
}

/**
 * @brief       TIM��ʼ��
 * 
 */
void TIM_Init(void)
{
	TIM_TimeBaseConfigure();/*Ҫ������ʱ�����������޷������ж�*/
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
		printf("TIMx1�ж�\n");/*���жϺ����о�����Ҫ����printf,�ú�������ʱ��ϳ����˴����ý�Ϊ��չ��ʵ������*/
		TIM_ClearFlag(Timx1, TIM_FLAG_Update);
	}
}

void TIM7_IRQHandler(void)
{
	if (TIM_GetFlagStatus(Timx2, TIM_FLAG_Update))
	{
		printf("TIMx2�ж�\n"); /*���жϺ����о�����Ҫ����printf,�ú�������ʱ��ϳ����˴����ý�Ϊ��չ��ʵ������*/
		TIM_ClearFlag(Timx2, TIM_FLAG_Update);
	}
}
