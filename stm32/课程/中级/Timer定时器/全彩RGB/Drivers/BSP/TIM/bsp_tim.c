#include "bsp_tim.h"
#include "bsp_led.h"

extern uint16_t value;
/**
 * @brief �ж����ȼ����ú���
 *
 */
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/*�������ȼ����飬ֻ������������������һ��*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = Timx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; /*��ռ���ȼ�*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  /*�����ȼ�*/
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief ��ʱ����ʼ������
 *
 */
void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	/*��ʱ��*/
	Timx_APBxClock_FUN(Timx_CLK, ENABLE);
	/*ѡ���ڲ�ʱ��*/
	TIM_InternalClockConfig(Timx);/*����ѡ��ʱ����Ĭ���ڲ�ʱ��*/
	/*ѡ������ʱ��*/

	/*������ʱ��û�еĹ��� */
	/*��ʱ��ʱ��CK_INTƵ���������˲�������Ƶ��֮��ķ�Ƶ��*/
	//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	/*�趨�ظ�������ֵ�����߼���ʱ���д˹���*/
	// TIM_TimeBaseInitStructure.TIM_RepetitionCounter;
	/*��װ�ؼĴ���������ʽΪ���ϼ���*/
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/*��װ�ؼĴ���ֵ*/
	TIM_TimeBaseInitStructure.TIM_Period = 7200 - 1;
	/*Ԥ��Ƶֵ*/
	TIM_TimeBaseInitStructure.TIM_Prescaler = 10000 - 1;
	TIM_TimeBaseInit(Timx, &TIM_TimeBaseInitStructure);
	/*����TIM_TimeBaseInit�����ز���һ�������¼���������ʱ��͸����ж�ͬʱ����������ʹ��TIM����������һ���ж�*/
	/*���������ʼʱ�����жϣ�ֻ����TIM_TimeBaseInit��TIM_ITConfig֮����������¼���־λ����*/
	TIM_ClearFlag(Timx, TIM_FLAG_Update); /*���д���ɱ�֤value��0��ʼ��*/
	/*TIM�ж������뿪�ж�*/
	NVIC_Configuration();
	TIM_ITConfig(Timx, TIM_IT_Update, ENABLE);
	/*ʹ��TIM*/
	TIM_Cmd(Timx, ENABLE);
}

// void TIM7_IRQHandler(void)
// {
// 	while (TIM_GetITStatus(Timx, TIM_IT_Update))
// 	{
// 		value++;
// 		TIM_ClearFlag(Timx, TIM_FLAG_Update);
// 		LEDG_TOGGLE;
// 	}
// }

