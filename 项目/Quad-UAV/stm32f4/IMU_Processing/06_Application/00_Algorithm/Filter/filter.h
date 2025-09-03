/**
  ******************************************************************************
  * @file    filter.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/30
  * @brief   �����˲��㷨
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

/*һ�׵�ͨ�˲���״̬�ṹ��*/
typedef struct
{
    float alpha; /*�˲�ϵ��*/
    float prev_output; /*��һ�����ֵ*/
    bool is_first_filter; /*�״��˲���־*/
} LPF1_State_t;

/*���׵�ͨ�˲���״̬�ṹ��(ֱ��II��)*/
typedef struct
{
    float alpha0; /*ǰ���˲�ϵ��*/
    float alpha1; /*�˲�ϵ��*/
    float alpha2; /*�˲�ϵ��*/
    float beta1; /*�����˲�ϵ��*/
    float beta2; /*�˲�ϵ��*/
    float delay_element1; /*�ӳ�Ԫ��*/
    float delay_element2; /*�ӳ�Ԫ��*/
} LPF2_State_t;

void LPF1_Init(LPF1_State_t *state, float cutoff_freq, float sample_freq);
float LPF1_Update(LPF1_State_t *state, float input);
void LPF2_Init(LPF2_State_t *state, float cutoff_freq, float sample_freq);
float LPF2_Update(LPF2_State_t *state, float input);

#endif /* __FILTER_H */

