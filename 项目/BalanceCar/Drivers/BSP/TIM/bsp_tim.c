/**
 * *****************************************************************************
 * @file        bsp_tim.c
 * @brief       tim驱动程序
 *              输出PWM波：TIM3_CH1(PA6)-PWMA、TIM3_CH2(PA7)-PWMB
 * @author      
 * @date        2024-11-22
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:    STM32F103VET6、TIM3_CH1(PA6)、CH2(PA7)
 *
 * *****************************************************************************
 */

#include "bsp_tim.h"

/*******************************配置输出比较输出PWM波****************************/
/**
 * @brief     TIM相关GPIO配置
 *
 */
static void Tim_GPIOConfigure(void)
{
    GPIO_InitTypeDef GPIOInitStructure;
    PWMx_TIM_GPIO_CLK_FUN(PWMx_TIM_GPIO_CLK, ENABLE);
    /*CH1(PA6)*/
    GPIOInitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIOInitStructure.GPIO_Pin = PWMA_TIM_GPIO_Pin;
    GPIOInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWMA_TIM_GPIO_Port, &GPIOInitStructure);
    /*CH2(PA7)*/
    GPIOInitStructure.GPIO_Pin = PWMB_TIM_GPIO_Pin;
    GPIO_Init(PWMB_TIM_GPIO_Port, &GPIOInitStructure);
}

/**
 * @brief       TIM时基初始化，配置PWM频率为10k
 *              PWM频率：72M/(ARR*PSC),一般电机要求PWM频率为几K~几十K，建议不要超过20K
 *              PWM频率过小电机会卡顿，噪声大，过大会使MOS管压力大
 *
 */
static void Tim_TimeBaseConfigure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    PWMx_TIM_CLK_FUN(PWMx_TIM_CLK, ENABLE);
		/*选择内部时钟*/
    TIM_InternalClockConfig(Timx);
    /*定时器时钟CK_INT频率与数字滤波器采样频率之间的分频比*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = (7200) - 1;/*PWM频率设置为10K*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(Timx, &TIM_TimeBaseInitStructure);
}

/**
 * @brief       输出比较配置，占空比计算：CRC/ARR
 *
 */
static void Tim_OCConfigure(void)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; /*可用于设置占空比，在别处设置*/
    PWMA_TIM_OCInitFUN(Timx, &TIM_OCInitStructure);
    PWMB_TIM_OCInitFUN(Timx, &TIM_OCInitStructure);
}

/**
 * @brief       输出比较初始化
 *
 */
void Tim_OCInit(void)
{
    Tim_TimeBaseConfigure();
    Tim_GPIOConfigure();
    Tim_OCConfigure();
    TIM_Cmd(Timx, ENABLE);
}
