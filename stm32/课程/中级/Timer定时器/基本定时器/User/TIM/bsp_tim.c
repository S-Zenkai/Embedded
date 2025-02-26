#include "bsp_tim.h"
#include "bsp_led.h"

extern uint16_t value;

#if 1
/**
 * @brief 中断优先级配置函数
 *
 */
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/*配置优先级分组，只需在整个工程中配置一次*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
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
	/*基本定时器没有的功能 */
	/*定时器时钟CK_INT频率与数字滤波器采样频率之间的分频比*/
	//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	/*设定重复计数器值，仅高级定时器有此功能*/
	// TIM_TimeBaseInitStructure.TIM_RepetitionCounter;
	/*重装载寄存器计数方式为向上计数*/
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/*重装载寄存器值*/
	TIM_TimeBaseInitStructure.TIM_Period = 7200 - 1;
	/*预分频值*/
	TIM_TimeBaseInitStructure.TIM_Prescaler = 10000 - 1;
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
  * 函    数：定时中断初始化
  * 参    数：无
  * 返 回 值：无
  */

void Timer_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//开启TIM2的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM2);		//选择TIM2为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;				//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;				//预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);				//将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元	
	
	/*中断输出配置*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);						//清除定时器更新标志位
																//TIM_TimeBaseInit函数末尾，手动产生了更新事件
																//若不清除此标志位，则开启中断后，会立刻进入一次中断
																//如果不介意此问题，则不清除此标志位也可
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);					//开启TIM2的更新中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//配置NVIC为分组2
																//即抢占优先级范围：0~3，响应优先级范围：0~3
																//此分组配置在整个工程中仅需调用一次
																//若有多个中断，可以把此代码放在main函数内，while循环之前
																//若调用多次配置分组的代码，则后执行的配置会覆盖先执行的配置
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//选择配置NVIC的TIM2线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);								//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*TIM使能*/
	TIM_Cmd(TIM2, ENABLE);			//使能TIM2，定时器开始运行
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)		//判断是否是TIM2的更新事件触发的中断
	{
		value ++;												//Num变量自增，用于测试定时中断
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);			//清除TIM2更新事件的中断标志位
															//中断标志位必须清除
															//否则中断将连续不断地触发，导致主程序卡死
	}
}
#endif
