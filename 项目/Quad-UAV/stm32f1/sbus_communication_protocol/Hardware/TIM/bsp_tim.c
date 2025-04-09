/**
  ******************************************************************************
  * @file    bsp_tim.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/03
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t overflow_counter = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void Timer_NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /*���ȼ���*/

    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  TIMʱ������
  * @note   
  * @param  ��
  * @retval ��
  */
static void Timer_TBConfigure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_InternalClockConfig(TIM4);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36-1;/*1us������+1*/
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void timer_init(void)
{
	Timer_NVIC_Configure();
    Timer_TBConfigure();
}

/**
  * @brief  ��ȡʱ�������
  * @note   ���ڿ����ڶ�ȡoverflow_counter��TIM4->CNT��ʱ�򴥷��жϵ��»�ȡʱ���������������������ٽ��������ȱ�����overflow_counter��CNT��
  * @param  ��
  * @retval ��
  */
uint64_t timer_GetTick(void)
{
    uint16_t cnt;
    uint32_t overflow;
    __disable_irq();/*�ٽ�����ʧ���ж�*/
    /*��ȡ���ֵ�����ֵ*/
    overflow = overflow_counter;
    cnt = TIM4->CNT;
    /*����ڶ�ȡcnt��overflow�������������overflow=1����cnt���=0�������һ���ж����ڵ�ʱ��*/
    /*�������Ƿ������������������˱����¶�ȡ*/
    if ((TIM4->SR & TIM_FLAG_Update) && (cnt < 0x8000))
    {
        cnt = TIM4->CNT;  // ���¶�ȡCNT
        overflow++;
    }
    __enable_irq();
    return ((uint64_t)overflow << 16) | cnt;
}

/**
  * @brief  TIM4�жϷ������
  * @note   
  * @param  ��
  * @retval ��
  */
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        overflow_counter++;
    }
}
