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

/*一阶低通滤波器参数结构体*/
typedef struct
{
    float Tc;/*时间常数*/
    float a;/*滤波系数*/
    float delay_element1;
}LPF1ordParam_t;

/*一阶低通滤波器参数结构体*/
typedef struct
{
    float sample_freq;/*采样频率*/
    float cutoff_freq;/*截至频率*/
    float delay_element1;/*直接II型延迟单元*/
    float delay_element2;
    float b0, b1, b2;/*前馈系数*/
    float a1, a2;/*反馈系数*/
}LPF2ordParam_t;

// /*滑动窗口限幅平均滤波参数*/
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
