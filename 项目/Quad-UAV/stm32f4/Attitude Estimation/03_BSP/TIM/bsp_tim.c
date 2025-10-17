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

/**
  * @brief  TIM6初始化配置
  * @note   
  * @param  无
  * @retval 无
  */
static void tim6_init(void)
{
    /*NVIC配置*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM6_DAC_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM6_DAC_IRQ_SUB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    /*时基配置*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM_InternalClockConfig(TIM6);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = TIM6_RELOAD_VALUE-1;/*1000hz中断一次*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = 90-1;/*APB1时钟频率45M*/
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

    /*使能中断与TIM6*/
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
}



void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // 使能GPIOB时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // 配置SCL引脚 (PB8)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // 输出模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // 开漏
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上下拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 高速
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}




/**
  * @brief  TIM初始化
  * @note   
  * @param  无
  * @retval 无
  */
void tim_init(void)
{
    gpio_init();
    tim6_init();
}


