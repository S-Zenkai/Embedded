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
/*��ѭ��Ƶ�ʸ��±�־λ*/
__IO FlagStatus Main_Loop_Update_Flag = RESET;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  TIM6��ʼ������
  * @note   
  * @param  ��
  * @retval ��
  */
static void tim6_init(void)
{
    /*NVIC����*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM6_DAC_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM6_DAC_IRQ_SUB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    /*ʱ������*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM_InternalClockConfig(TIM6);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = TIM6_RELOAD_VALUE-1;/*1000hz�ж�һ��*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = 90-1;/*APB1ʱ��Ƶ��45M*/
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

    /*ʹ���ж���TIM6*/
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
}



void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // ʹ��GPIOBʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // ����SCL���� (PB8)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // ���ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // ��©
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // ��������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // ����
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}




/**
  * @brief  TIM��ʼ��
  * @note   
  * @param  ��
  * @retval ��
  */
void tim_init(void)
{
    gpio_init();
    tim6_init();
}


