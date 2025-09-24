/**
  ******************************************************************************
  * @file    filter.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   通用滤波器算法实现
  ******************************************************************************
  * @attention
  *
  * 本文件包含一阶低通滤波器的实现。
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "filter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 初始化一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param dt 采样时间 (秒)
 * @param cutoff_freq 截止频率 (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq)
{
    if (cutoff_freq <= 0.0f) {
        filter->alpha = 1.0f; // 如果截止频率无效，则不滤波
    } else {
        float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
        filter->alpha = dt / (dt + rc);
    }
    filter->last_output = 0.0f;
}

/**
 * @brief 应用一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param input 当前的输入值
 * @return 滤波后的输出值
 */
float lpf_apply(LPF_t *filter, float input)
{
    // 公式: output = last_output + alpha * (input - last_output)
    float output = filter->last_output + filter->alpha * (input - filter->last_output);
    filter->last_output = output;
    return output;
}