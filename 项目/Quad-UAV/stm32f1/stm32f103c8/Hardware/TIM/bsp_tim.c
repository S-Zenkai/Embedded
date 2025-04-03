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
    /*优先级组*/

    /*USART1*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_InternalClockConfig(TIM4);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36-1;/*1us计数器+1*/
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
  * @brief  获取时间戳函数
  * @note   由于可能在读取overflow_counter或TIM4->CNT的时候触发中断导致获取时间戳错误，所以这里加入了临界区，事先保存了overflow_counter和CNT，
  * @param  无
  * @retval 无
  */
uint64_t timer_GetTick(void)
{
    uint16_t cnt;
    uint32_t overflow;
    __disable_irq();/*临界区，失能中断*/
    /*读取溢出值与计数值*/
    overflow = overflow_counter;
    cnt = TIM4->CNT;
    /*如果在读取cnt或overflow后发生了溢出，如overflow=1，后cnt溢出=0，便会少一个中断周期的时间*/
    /*这里检测是否发生了溢出，如果发生了便重新读取*/
    if ((TIM4->SR & TIM_FLAG_Update) && (cnt < 0x8000))
    {
        cnt = TIM4->CNT;  // 重新读取CNT
        overflow++;
    }
    __enable_irq();
    return ((uint64_t)overflow << 16) | cnt;
}

/**
  * @brief  TIM4中断服务程序
  * @note   
  * @param  无
  * @retval 无
  */
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        overflow_counter++;
    }
}
