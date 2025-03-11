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
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  TIM时基配置
  * @note   
  * @param  无
  * @retval 无
  */
static void Timer_TBConfigure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM_InternalClockConfig(TIM6);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 45-1;/*APB1时钟频率45M*/
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
}

void timer_tbinit(void)
{
    Timer_TBConfigure();
    TIM_Cmd(TIM6, ENABLE);
}

void timer_ti_init(void)
{
    Timer_TBConfigure();
    NVIC_Configure();
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update)==SET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}
