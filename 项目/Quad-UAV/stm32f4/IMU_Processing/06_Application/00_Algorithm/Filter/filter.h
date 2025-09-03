/**
  ******************************************************************************
  * @file    filter.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/30
  * @brief   各种滤波算法
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stdbool.h"

#ifndef NULL
#define NULL 0
#endif

/*一阶低通滤波器状态结构体*/
typedef struct
{
    float alpha; /*滤波系数*/
    float prev_output; /*上一个输出值*/
    bool is_first_filter; /*首次滤波标志*/
} LPF1_State_t;

/*二阶低通滤波器状态结构体(直接II型)*/
typedef struct
{
    float alpha0; /*前向滤波系数*/
    float alpha1; /*滤波系数*/
    float alpha2; /*滤波系数*/
    float beta1; /*反馈滤波系数*/
    float beta2; /*滤波系数*/
    float delay_element1; /*延迟元素*/
    float delay_element2; /*延迟元素*/
} LPF2_State_t;

void LPF1_Init(LPF1_State_t *state, float cutoff_freq, float sample_freq);
float LPF1_Update(LPF1_State_t *state, float input);
void LPF2_Init(LPF2_State_t *state, float cutoff_freq, float sample_freq);
float LPF2_Update(LPF2_State_t *state, float input);

#endif /* __FILTER_H */

