/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/06
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_exti.h"
#include "bsp_usart.h"
#include "bsp_mpu6000.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// extern float current_value;
// extern float dt;
uint8_t mpu6000ready_flag;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void NVIC_EXTI_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

static void EXTI_GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;/*EXTI≈‰÷√œ¬Ωµ—ÿ¥•∑¢*/
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource15);
    EXTI_InitStruct.EXTI_Line = EXTI_Line15;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
}

void exti_init(void)
{
    NVIC_EXTI_Configure();
    EXTI_GPIO_Configure();
    EXTI_Configure();
}



void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line15)==SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);
        mpu6000ready_flag = 1;
        // current_value = TIM_GetCounter(TIM6);
        // dt = current_value / 1000000;
        // TIM_SetCounter(TIM6, 0);
    }
}




