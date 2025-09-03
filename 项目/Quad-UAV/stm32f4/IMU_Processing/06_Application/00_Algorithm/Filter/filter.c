/**
  ******************************************************************************
  * @file    filter.c
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
#include "filter.h"
#include "pro_common.h"
#include <math.h>

/**
  * @brief  一阶低通滤波初始化
  * @note   
  * @param  state: 滤波器状态结构体
  * @param  cutoff_freq: 截止频率
  * @param  sample_freq: 采样频率
  * @retval 无
  */
void LPF1_Init(LPF1_State_t *state, float cutoff_freq, float sample_freq)
{
    if (state == NULL|| cutoff_freq <= 0 || sample_freq <= 0)
    {
        return; // 参数错误
    }
    state->alpha = 1.0f / (1.0f + 2.0f * PI * cutoff_freq / sample_freq);
    state->is_first_filter = true;
}

/**
  * @brief  一阶低通滤波
  * @note   
  * @param  state: 滤波器状态结构体指针
  * @param  input: 输入数据
  * @retval 滤波后的数据
  */
float LPF1_Update(LPF1_State_t *state, float input)
{
    float output;
    if (state == NULL)
    {
        return input; // 参数错误
    }
    /*首次滤波*/
    if (state->is_first_filter == true)
    {
        /*防止初始时输入过大造成的滤波输出跳变*/
        state->prev_output = input;
        state->is_first_filter = false;
    }
    output = state->alpha * input + (1.0f - state->alpha) * state->prev_output;
    state->prev_output = output;
    return output;
}

/**
  * @brief  二阶IIR低通滤波(直接II型)初始化
  * @note   
  * @param  state: 滤波器状态
  * @param  cutoff_freq: 截止频率
  * @param  sample_freq: 采样频率
  * @retval 无
  */
void LPF2_Init(LPF2_State_t *state, float cutoff_freq, float sample_freq)
{
    if (state == NULL|| cutoff_freq <= 0 || sample_freq <= 0)
    {
        return; // 参数错误
    }
    /*中间变量*/
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(PI / fr);/*频率预翘曲产生参数*/
    float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
    state->alpha0 = ohm * ohm / c;
    state->alpha1 = 2.0f * state->alpha0;
    state->alpha2 = state->alpha0;
    state->beta1 = 2.0f * (ohm * ohm - 1.0f) / c;
    state->beta2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
    state->delay_element1 = 0.0f;
    state->delay_element2 = 0.0f;
}

/**
  * @brief  二阶IIR低通滤波(直接II型)
  * @note   
  * @param  state: 滤波器状态结构体指针
  * @param  input: 输入数据
  * @retval 滤波后的数据
  */
float LPF2_Update(LPF2_State_t *state, float input)
{
    float output;
    if (state == NULL)
    {
        return input; // 参数错误
    }
    float delay_element0=input-state->beta1*state->delay_element1-state->beta2*state->delay_element2;
    /*不允许坏值通过*/
    /*防止数值不稳定等导致的滤波无穷大，如果是则将element0设置为input，代表重置滤波器*/
    if(!isfinite(delay_element0))
    {
        delay_element0 = input;
    }
    output = state->alpha0*delay_element0+state->alpha1*state->delay_element1+state->alpha2*state->delay_element2;
    state->delay_element2 = state->delay_element1;
    state->delay_element1 = delay_element0;
    return output;
}


