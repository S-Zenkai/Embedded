/**
  ******************************************************************************
  * @file    filter.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   通用滤波器算法头文件
  ******************************************************************************
  * @attention
  *
  * 本文件包含一阶低通滤波器的实现。
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 一阶低通滤波器结构体
 */
typedef struct {
    float alpha;          // 滤波系数, alpha = dt / (dt + RC), 其中 RC = 1 / (2*PI*cutoff_freq)
    float last_output;    // 上一次的滤波输出
} LPF_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief 初始化一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param dt 采样时间 (秒)
 * @param cutoff_freq 截止频率 (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq);

/**
 * @brief 应用一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param input 当前的输入值
 * @return 滤波后的输出值
 */
float lpf_apply(LPF_t *filter, float input);

#endif /* __FILTER_H */