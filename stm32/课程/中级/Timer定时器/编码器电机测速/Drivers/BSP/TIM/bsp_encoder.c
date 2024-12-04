/**
 * *****************************************************************************
 * @file        bsp_encoder.c
 * @brief       TIM编码器模式测量电机转速
 * @author      
 * @date        2024-11-22
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:    stm32f103vet6、tim编码器模式：tim4CH1、CH2
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_encoder.h"
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
    TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*这里可不配置，已由编码器配置托管，这里没什么用*/
    TimBaseInitStructure.TIM_Period = 65535 - 1;             /*ARR重装载寄存器值*/
    TimBaseInitStructure.TIM_Prescaler = 1 - 1;/*预分频值*/
    TimBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(Timx, &TimBaseInitStructure);
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
    /*关于gpio配置，当外部输入默认电平为高电平时可配置为上拉输入*/
    /*当外部输入默认电平为低电平时可配置为下拉输入*/
    /*不确定外部输入，可配置为浮空输入，缺点在于易受噪声干扰*/
    GPIOInitStructuer.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIOInitStructuer.GPIO_Pin = Timx_CH1_GPIO_Pin;
    GPIOInitStructuer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Timx_CH1_GPIO_Port, &GPIOInitStructuer);
    /*CH2配置*/
    GPIOInitStructuer.GPIO_Pin = Timx_CH2_GPIO_Pin;
    GPIO_Init(Timx_CH2_GPIO_Port, &GPIOInitStructuer);
}

/**
 * @brief       输入捕获初始化
 * 
 */
static void ICInit(void)
{
    TIM_ICInitTypeDef TimICInitStructure;
    TimICInitStructure.TIM_Channel = TIM_Channel_1;/*输入通道选择*/
    TimICInitStructure.TIM_ICFilter = 0xF;/*输入滤波器*/
    /*下面并非配置一般输入捕获上升沿触发，而是极性不反转，可不配置，在编码器配置函数中重复定义*/
    TimICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
    //TimICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;/*输入捕获预分频器，编码器模式未使用*/
    TimICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;/*选择TIxFPx*/
    TIM_ICInit(Timx ,&TimICInitStructure);

    TimICInitStructure.TIM_Channel = TIM_Channel_2; /*输入通道选择*/
    TimICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(Timx, &TimICInitStructure);
}

void EncoderInit(void)
{
    TimBaseInit();        /*时基单元初始化*/
    TimGPIOInit();        /*GPIO端口初始化*/
    ICInit();             /*输入捕获初始化*/
    /*配置编码器模式，配置TI1、TI2极性不反转*/
    TIM_EncoderInterfaceConfig(Timx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(Timx, ENABLE);
}

/**
 * @brief       测量输入信号频率，使用测周法，公式：fx=fc/N，其中，fc为标准频率，也是本文件时基设定的定时器计数频率(72M/PSC)
 * 
 * @return      uint32_t ,频率
 */
int16_t GetSpeed(void)
{
    uint16_t speed = 0;
    speed = TIM_GetCounter(Timx) / 1320;/*电机一圈11个脉冲，一个脉冲计4次数*/
    TIM_SetCounter(Timx, 0);
    return speed;
}
    /*------------------------------------test------------------------------------*/
