/**
 * *****************************************************************************
 * @file        bsp_tim.c
 * @brief       tim��������
 *              ���PWM����TIM3_CH1(PA6)-PWMA��TIM3_CH2(PA7)-PWMB
 * @author      
 * @date        2024-11-22
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:    STM32F103VET6��TIM3_CH1(PA6)��CH2(PA7)
 *
 * *****************************************************************************
 */

#include "bsp_tim.h"

/*******************************��������Ƚ����PWM��****************************/
/**
 * @brief     TIM���GPIO����
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
 * @brief       TIMʱ����ʼ��������PWMƵ��Ϊ10k
 *              PWMƵ�ʣ�72M/(ARR*PSC),һ����Ҫ��PWMƵ��Ϊ��K~��ʮK�����鲻Ҫ����20K
 *              PWMƵ�ʹ�С����Ῠ�٣������󣬹����ʹMOS��ѹ����
 *
 */
static void Tim_TimeBaseConfigure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    PWMx_TIM_CLK_FUN(PWMx_TIM_CLK, ENABLE);
		/*ѡ���ڲ�ʱ��*/
    TIM_InternalClockConfig(Timx);
    /*��ʱ��ʱ��CK_INTƵ���������˲�������Ƶ��֮��ķ�Ƶ��*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = (7200) - 1;/*PWMƵ������Ϊ10K*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(Timx, &TIM_TimeBaseInitStructure);
}

/**
 * @brief       ����Ƚ����ã�ռ�ձȼ��㣺CRC/ARR
 *
 */
static void Tim_OCConfigure(void)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; /*����������ռ�ձȣ��ڱ�����*/
    PWMA_TIM_OCInitFUN(Timx, &TIM_OCInitStructure);
    PWMB_TIM_OCInitFUN(Timx, &TIM_OCInitStructure);
}

/**
 * @brief       ����Ƚϳ�ʼ��
 *
 */
void Tim_OCInit(void)
{
    Tim_TimeBaseConfigure();
    Tim_GPIOConfigure();
    Tim_OCConfigure();
    TIM_Cmd(Timx, ENABLE);
}
