#include "bsp_tim.h"
#include "bsp_led.h"

extern uint16_t value;

#if 1
/**
 * @brief �ж����ȼ����ú���
 *
 */
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/*�������ȼ����飬ֻ������������������һ��*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
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

void TIM7_IRQHandler(void)
{
	while (TIM_GetITStatus(Timx, TIM_IT_Update))
	{
		value++;
		TIM_ClearFlag(Timx, TIM_FLAG_Update);
		LEDG_TOGGLE;
	}
}

// void TIM7_IRQHandler(void)
//{
//     while (TIM_GetITStatus(Timx, TIM_IT_Update))
//     {
//         value++;
//         TIM_ClearFlag(Timx, TIM_FLAG_Update);
////        LEDG_TOGGLE;
//    }
//}

#endif

#if 0
/**
  * ��    ������ʱ�жϳ�ʼ��
  * ��    ������
  * �� �� ֵ����
  */

void Timer_Init(void)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//����TIM2��ʱ��
	
	/*����ʱ��Դ*/
	TIM_InternalClockConfig(TIM2);		//ѡ��TIM2Ϊ�ڲ�ʱ�ӣ��������ô˺�����TIMĬ��ҲΪ�ڲ�ʱ��
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;				//�������ڣ���ARR��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;				//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//�ظ����������߼���ʱ���Ż��õ�
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);				//���ṹ���������TIM_TimeBaseInit������TIM2��ʱ����Ԫ	
	
	/*�ж��������*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);						//�����ʱ�����±�־λ
																//TIM_TimeBaseInit����ĩβ���ֶ������˸����¼�
																//��������˱�־λ�������жϺ󣬻����̽���һ���ж�
																//�������������⣬������˱�־λҲ��
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);					//����TIM2�ĸ����ж�
	
	/*NVIC�жϷ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//����NVICΪ����2
																//����ռ���ȼ���Χ��0~3����Ӧ���ȼ���Χ��0~3
																//�˷������������������н������һ��
																//���ж���жϣ����԰Ѵ˴������main�����ڣ�whileѭ��֮ǰ
																//�����ö�����÷���Ĵ��룬���ִ�е����ûḲ����ִ�е�����
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;						//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//ѡ������NVIC��TIM2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//ָ��NVIC��·����ռ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);								//���ṹ���������NVIC_Init������NVIC����
	
	/*TIMʹ��*/
	TIM_Cmd(TIM2, ENABLE);			//ʹ��TIM2����ʱ����ʼ����
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)		//�ж��Ƿ���TIM2�ĸ����¼��������ж�
	{
		value ++;												//Num�������������ڲ��Զ�ʱ�ж�
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);			//���TIM2�����¼����жϱ�־λ
															//�жϱ�־λ�������
															//�����жϽ��������ϵش�����������������
	}
}
#endif
