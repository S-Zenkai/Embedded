/**
 * *****************************************************************************
 * @file        bsp_encoder.h
 * @brief       
 * @author      
 * @date        2024-11-22
 * @copyright   
 * *****************************************************************************
 */
#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*TIM时钟*/
#define             Timx_APBxClock_FUN          RCC_APB1PeriphClockCmd
#define             Timx_CLK                    RCC_APB1Periph_TIM4
/*TIM*/
#define             Timx                        TIM4
/*TIM_GPIO时钟*/
#define             Timx_GPIO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define             Timx_GPIO_CLK               RCC_APB2Periph_GPIOB
/*TIM_GPIO(宏Timx对应的GPIO)*/
#define             Timx_CH1_GPIO_Port          GPIOB
#define             Timx_CH1_GPIO_Pin           GPIO_Pin_6
#define             Timx_CH2_GPIO_Port          GPIOB
#define             Timx_CH2_GPIO_Pin           GPIO_Pin_7
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void EncoderInit(void);
int16_t GetSpeed(void);
/*------------------------------------test------------------------------------*/

#endif /* __BSP_ENCODER_H */
