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
#ifndef __FILTER1_H
#define __FILTER1_H
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

#endif /* __FILTER1_H */

