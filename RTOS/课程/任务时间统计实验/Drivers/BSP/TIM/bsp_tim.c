#include "bsp_tim.h"
#include "bsp_led.h"
#include "bsp_usart.h"

extern uint16_t value;
/**
 * @brief 中断优先级配置函数
 *
 */
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/*配置优先级分组，只需在整个工程中配置一次*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = Timx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; /*抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  /*子优先级*/
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief 定时器初始化函数
 *
 */
void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	/*开时钟*/
	Timx_APBxClock_FUN(Timx_CLK, ENABLE);
	/*选择内部时钟*/
	TIM_InternalClockConfig(Timx);/*若不选择时钟则默认内部时钟*/
	/*选择其他时钟*/

	/*重装载寄存器计数方式为向上计数*/
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/*重装载寄存器值*/
	TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;
	/*预分频值*/
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInit(Timx, &TIM_TimeBaseInitStructure);
	/*由于TIM_TimeBaseInit在最后回产生一个更新事件，而更新时间和更新中断同时发生，故在使能TIM会立即进入一次中断*/
	/*若不想程序开始时进入中断，只需在TIM_TimeBaseInit与TIM_ITConfig之间清楚更新事件标志位即可*/
	TIM_ClearFlag(Timx, TIM_FLAG_Update); /*这行代码可保证value从0开始计*/
	/*TIM中断配置与开中断*/
	NVIC_Configuration();
	TIM_ITConfig(Timx, TIM_IT_Update, ENABLE);
	/*使能TIM*/
	TIM_Cmd(Timx, ENABLE);
}

uint32_t FreeRTOSRunTimeTicks;
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(Timx, TIM_IT_Update)==SET)
    {
        FreeRTOSRunTimeTicks++;
        // printf("count=%d", Timer_Conter);
        TIM_ClearFlag(Timx, TIM_FLAG_Update);
    }
}

/**
 * @brief       使用freertos任务时间统计功能时，用于配置定时器
 * 
 */
void ConfigureTimeForRunTimeStats(void)
{
    Timer_Init();
    FreeRTOSRunTimeTicks = 0;
}

