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
#include <stdbool.h>
#include "pro_common.h"
#include "bsp_systick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t overflow_counter = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/****************************TIM1����*******************************************/
/** @addtogroup TIM1����
 * @{
 */
//static void Timer1_NVIC_Configure(void)
//{
//    NVIC_InitTypeDef NVIC_InitStructure;
//    /*���ȼ���*/
//    /*USART1*/
//    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_Init(&NVIC_InitStructure);
//}

/**
 * @brief  TIMʱ������
 * @note
 * @param  ��
 * @retval ��
 */
//static void Timer1_TBConfigure(void)
//{
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//    TIM_InternalClockConfig(TIM1);
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1; /*1us������+1*/
//    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
//    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
//    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
//    TIM_Cmd(TIM1, ENABLE);
//}

/**
 * @brief  TIM4�жϷ������
 * @note
 * @param  ��
 * @retval ��
 */
//void TIM1_UP_IRQHandler(void)
//{
//    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
//    {
//        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//        overflow_counter++;
//    }
//}

/**
 * @}
 */

/****************************TIM2-TIM4����*******************************************/
/** @addtogroup TIM2-TIM4����
 * @{
 */

static void Timer2_4_GPIOConfigure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    /*PA0-TIM2CH1-IO_MO1*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*PA1-TIM2CH2-IO_MO2*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*PB8-TIM4CH3-IO_MO3*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*PB9-TIM4CH4-IO_MO4*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*PA6-TIM3CH1-IO_MO5*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*PA7-TIM3CH2-IO_MO6*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*PB0-TIM3CH3-IO_MO7*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*PB1-TIM3CH4-IO_MO8*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void Timer2_4_TBConfigure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    TIM_InternalClockConfig(TIM2);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 2500-1;      /*400hz pwmƵ�ʣ���pix4��ͬ*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;   /*ע�⣬TIMʱ�Ӳ����ܵ�������ʱ��;1us��������Ͻ�����1000us-2000us��Χ*/
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_InternalClockConfig(TIM3);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_InternalClockConfig(TIM4);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
}

static void Timer2_4_OCConfigure(void)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; /*�ߵ�ƽΪ��Ч��ƽ*/
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; /*�ߵ�ƽΪ��Ч��ƽ*/
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; /*ȷ����ʼʱ�����ת*/
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @}
 */

void timer_init(void)
{
//    Timer1_NVIC_Configure();
//    Timer1_TBConfigure();

    Timer2_4_GPIOConfigure();
    Timer2_4_TBConfigure();
    Timer2_4_OCConfigure();
}


void set_pwm(uint16_t *pwm_buff)
{
    // uint8_t ii = 0;
    // uint8_t chan = (channel <= RC_PWM_CHANNELS) ? channel : RC_PWM_CHANNELS;
    uint16_t pwm = pwm_buff[1];
    TIM_SetCompare1(TIM2, pwm); /*ͨ��1*/
    TIM_SetCompare2(TIM2, pwm);
    TIM_SetCompare3(TIM4, pwm);
    TIM_SetCompare4(TIM4, pwm);
    TIM_SetCompare1(TIM3, pwm);
    TIM_SetCompare2(TIM3, pwm);
    TIM_SetCompare3(TIM3, pwm);
    TIM_SetCompare4(TIM3, pwm);
}

/**
  * @brief  ʹ���ͣת
  * @note   
  * @param  ��
  * @retval ��
  */
void motor_stop(void)
{
    TIM_SetCompare1(TIM2, PWM_TARGRT_MIN); /*ͨ��1*/
    TIM_SetCompare2(TIM2, PWM_TARGRT_MIN);
    TIM_SetCompare3(TIM4, PWM_TARGRT_MIN);
    TIM_SetCompare4(TIM4, PWM_TARGRT_MIN);
    TIM_SetCompare1(TIM3, PWM_TARGRT_MIN);
    TIM_SetCompare2(TIM3, PWM_TARGRT_MIN);
    TIM_SetCompare3(TIM3, PWM_TARGRT_MIN);
    TIM_SetCompare4(TIM3, PWM_TARGRT_MIN);
}

// void motor_init(void)
// {
//     Timer2_4_GPIOConfigure();
//     Timer2_4_TBConfigure();
//     Timer2_4_OCConfigure();

//     TIM_SetCompare1(TIM2, PWM_TARGRT_MAX); /*ͨ��1*/
//     TIM_SetCompare2(TIM2, PWM_TARGRT_MAX);
//     TIM_SetCompare3(TIM4, PWM_TARGRT_MAX);
//     TIM_SetCompare4(TIM4, PWM_TARGRT_MAX);
//     TIM_SetCompare1(TIM3, PWM_TARGRT_MAX);
//     TIM_SetCompare2(TIM3, PWM_TARGRT_MAX);
//     TIM_SetCompare3(TIM3, PWM_TARGRT_MAX);
//     TIM_SetCompare4(TIM3, PWM_TARGRT_MAX);
//     delay_ms(2000);
//     TIM_SetCompare1(TIM2, PWM_TARGRT_MIN); /*ͨ��1*/
//     TIM_SetCompare2(TIM2, PWM_TARGRT_MIN);
//     TIM_SetCompare3(TIM4, PWM_TARGRT_MIN);
//     TIM_SetCompare4(TIM4, PWM_TARGRT_MIN);
//     TIM_SetCompare1(TIM3, PWM_TARGRT_MIN);
//     TIM_SetCompare2(TIM3, PWM_TARGRT_MIN);
//     TIM_SetCompare3(TIM3, PWM_TARGRT_MIN);
//     TIM_SetCompare4(TIM3, PWM_TARGRT_MIN);
//     delay_ms(1000);
// }

// /**
//  * @brief  ��ȡʱ�������
//  * @note   ���ڿ����ڶ�ȡoverflow_counter��TIM4->CNT��ʱ�򴥷��жϵ��»�ȡʱ���������������������ٽ��������ȱ�����overflow_counter��CNT��
//  * @param  ��
//  * @retval ��
//  */
// uint64_t timer_GetTick(void)
// {
//     uint16_t cnt;
//     uint32_t overflow;
//     __disable_irq(); /*�ٽ�����ʧ���ж�*/
//     /*��ȡ���ֵ�����ֵ*/
//     overflow = overflow_counter;
//     cnt = TIM1->CNT;
//     /*����ڶ�ȡcnt��overflow�������������overflow=1����cnt���=0�������һ���ж����ڵ�ʱ��*/
//     /*�������Ƿ������������������˱����¶�ȡ*/
//     if ((TIM1->SR & TIM_FLAG_Update) && (cnt < 0x8000))
//     {
//         cnt = TIM1->CNT; // ���¶�ȡCNT
//         overflow++;
//     }
//     __enable_irq();
//     return ((uint64_t)overflow << 16) | cnt;
// }
