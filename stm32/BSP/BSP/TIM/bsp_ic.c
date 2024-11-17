/**
 * *****************************************************************************
 * @file        bsp_ic.c
 * @brief       tim输入捕获实验，通过输入捕获功能测量pwm信号频率和占空比
 * @author      
 * @date        2024-11-12
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:    stm32f103vet6、输入捕获：tim4。产生pwm波：tim3
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_ic.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief       TIM时基初始化，
 * 
 */
static void TimBaseInit(void)
{
    TIM_TimeBaseInitTypeDef TimBaseInitStructure;
    Timx_APBxClock_FUN(Timx_CLK, ENABLE);
    /*选择内部时钟*/
    TIM_InternalClockConfig(Timx);
    TimBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; /*定时器时钟CK_INT频率与数字滤波器采样频率之间的分频比*/
    TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/**/
    TimBaseInitStructure.TIM_Period = 65535 - 1;             /*ARR重装载寄存器值*/
    TimBaseInitStructure.TIM_Prescaler = 72 - 1;/*预分频值*/
    TimBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(Timx, &TimBaseInitStructure);
    /*由于TIM_TimeBaseInit在最后回产生一个更新事件，而更新时间和更新中断同时发生，故在使能TIM会立即进入一次中断*/
    /*若不想程序开始时进入中断，只需在TIM_TimeBaseInit与TIM_ITConfig之间清楚更新事件标志位即可*/
    TIM_ClearFlag(Timx, TIM_FLAG_Update); /*这行代码可保证value从0开始计*/
}

/**
 * @brief       与TIM_CH相关的GPIO初始化配置
 * 
 */
static void TimGPIOInit(void)
{
    GPIO_InitTypeDef GPIOInitStructuer;
    Timx_GPIO_APBxClock_FUN(Timx_GPIO_CLK, ENABLE);
    /*CH1，配置为输入捕获通道*/
    GPIOInitStructuer.GPIO_Mode = GPIO_Mode_IPU;
    GPIOInitStructuer.GPIO_Pin = Timx_CH1_GPIO_Pin;
    GPIOInitStructuer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Timx_CH1_GPIO_Port, &GPIOInitStructuer);
}

/**
 * PWM各参数计算公式：
 *      1.PWM频率：F=CK_PSC/(ARR+1)(PSC+1);其中，CK_PSC是分配后时钟频率，ARR是重装载寄存器中的值，PSC是分频系数
 *      2.占空比：Duty=CRR/(ARR+1);CRR是捕获比较寄存器中的值
 *      3.分辨率：Reso=1/(ARR+1)
 *
 */

/**
 * @brief       输入捕获初始化
 * 
 */
static void ICInit(void)
{
    TIM_ICInitTypeDef TimICInitStructure;
    TimICInitStructure.TIM_Channel = TIM_Channel_1;/*输入通道选择*/
    TimICInitStructure.TIM_ICFilter = 0xF;/*输入滤波器*/
    TimICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;/*边沿触发选择*/
    TimICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;/*输入捕获预分频器*/
    TimICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;/*选择TIxFPx*/
    TIM_ICInit(Timx ,&TimICInitStructure);

    /*PWMI配置*/
    TIM_PWMIConfig(Timx, &TimICInitStructure);
    TIM_ICInit(Timx, &TimICInitStructure);

    /*从模式配置*/
    /*实现计数器自动清零*/
    TIM_SelectInputTrigger(Timx, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(Timx, TIM_SlaveMode_Reset);
}

void TimICInit(void)
{
    TimBaseInit();        /*时基单元初始化*/
    TimGPIOInit();        /*GPIO端口初始化*/
    ICInit();          /*输入捕获初始化*/
    TIM_Cmd(Timx, ENABLE);
}

/**
 * @brief       测量输入信号频率，使用测周法，公式：fx=fc/N，其中，fc为标准频率，也是本文件时基设定的定时器计数频率(72M/PSC)
 * 
 * @return      uint32_t ,频率
 */
uint16_t GetFreq(void)
{
    return 1000000 / (TIM_GetCapture1(Timx)+1);
}

/**
 * @brief       Get the Duty object
 * 
 * @return      uint8_t 占空比
 */
uint8_t GetDuty(void)
{
    return ((TIM_GetCapture2(Timx) + 1) * 100) / (TIM_GetCapture1(Timx) + 1);
}

#if 0
/**
 * @brief       测频法测量频率，公式：fc=T/N；其中，T为闸门时间（本函数通过定时中断功能实现）；
 *              N为T时间内待测信号上升沿数；
 * @note        使用本函数测量频率还需配置TIM4定时中断功能以及定义相关的变量
 * 
 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetFlagStatus(Timx, TIM_FLAG_Update))
    {
        freq = counter / 6;
        counter = 0;
        OLED_Printf(0, 0, OLED_8X16, "freq=%d", freq);
        OLED_Update();
        TIM_ClearFlag(Timx, TIM_FLAG_Update);
    }
    if (TIM_GetFlagStatus(Timx, TIM_FLAG_CC1))
    {
        counter++;
        TIM_ClearFlag(Timx, TIM_FLAG_CC1);
    }
}
#endif


    /*------------------------------------test------------------------------------*/
