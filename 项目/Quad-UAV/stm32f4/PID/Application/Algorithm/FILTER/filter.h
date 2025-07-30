/**
  ******************************************************************************
  * @file    filter.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/02
  * @brief   
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

/* Exported types ------------------------------------------------------------*/

/*һ�׵�ͨ�˲��������ṹ��*/
typedef struct
{
    float Tc;/*ʱ�䳣��*/
    float a;/*�˲�ϵ��*/
    float delay_element1;
}LPF1ordParam_t;

/*һ�׵�ͨ�˲��������ṹ��*/
typedef struct
{
    float sample_freq;/*����Ƶ��*/
    float cutoff_freq;/*����Ƶ��*/
    float delay_element1;/*ֱ��II���ӳٵ�Ԫ*/
    float delay_element2;
    float b0, b1, b2;/*ǰ��ϵ��*/
    float a1, a2;/*����ϵ��*/
}LPF2ordParam_t;

// /*���������޷�ƽ���˲�����*/
// typedef struct
// {
//     uint8_t win_sz;
//     float win_buf[win_sz];
// } MoveAver_t;

/* Exported contants --------------------------------------------------------*/



/* Exported macro ------------------------------------------------------------*/

// #define         PI                  3.1415926f

/* Exported functions ------------------------------------------------------- */

void LPF1ord_Init(LPF1ordParam_t *LPF1ordParam, uint16_t sample_freq, uint16_t cutoff_freq);
float LPF1ord(LPF1ordParam_t *LPF1ordParam, float invalue, float out_value_);
void LPF2ord_Init(LPF2ordParam_t *LPF2ordParam, float sample_freq, float cutoff_freq);
float LPF2ord(LPF2ordParam_t* LPF2ordParam,float simple);
float moving_average(float in);
#endif /* __FILTER_H */
