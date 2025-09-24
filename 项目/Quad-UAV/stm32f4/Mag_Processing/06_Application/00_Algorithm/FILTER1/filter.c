/**
 ******************************************************************************
 * @file    filter.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/03/02
 * @brief   滤波函数
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "filter.h"
#include <math.h>
#include "bsp_usart.h"
#include <stdbool.h>
#include "pro_common.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*一阶低通滤波器参数*/
LPF1ordParam_t LPF1ordParam;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  设置一阶低通滤波器滤波参数
 * @note
 * @param  LPF1ordParam：参数结构体变量;
 * @param  sample_freq：采样频率;
 * @param  cutoff_freq：截止频率;
 * @retval 无
 */
static void LPF1ord_SetParam(LPF1ordParam_t *LPF1ordParam, uint16_t sample_freq, uint16_t cutoff_freq)
{
    LPF1ordParam->Tc = 1 / (2 * PI * cutoff_freq);
    LPF1ordParam->a = 1 / (1 + LPF1ordParam->Tc * sample_freq);
}

/**
 * @brief  初始化一阶低通滤波器参数
 * @note
 * @param  LPF1ordParam：参数结构体变量;
 * @param  sample_freq：采样频率;
 * @param  cutoff_freq：截止频率;
 * @retval 无
 */
void LPF1ord_Init(LPF1ordParam_t *LPF1ordParam, uint16_t sample_freq, uint16_t cutoff_freq)
{
    LPF1ord_SetParam(LPF1ordParam, sample_freq, cutoff_freq);
}

/**
 * @brief  一阶低通滤波器函数
 * @note
 * @param  LPF1ordParam：参数结构体变量;
 * @param  invalue：输入值
 * @param  out_value_：上一时刻输出值
 * @retval 滤波值
 */
float LPF1ord(LPF1ordParam_t *LPF1ordParam, float invalue, float out_value_)
{
    return LPF1ordParam->a * invalue + (1 - LPF1ordParam->a) * out_value_;
}

/**
 * @brief  设置二阶低通滤波器滤波参数
 * @note
 * @param  LPF2ordParam：参数结构体变量
 * @retval 无
 */
static void LPF2ord_SetParam(LPF2ordParam_t *LPF2ordParam)
{
    float fr = LPF2ordParam->sample_freq / LPF2ordParam->cutoff_freq;
    float ohm = tanf(PI / fr);/*频率预翘曲产生参数*/
    float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
    LPF2ordParam->b0 = ohm * ohm / c;
    LPF2ordParam->b1 = 2.0f * LPF2ordParam->b0;
    LPF2ordParam->b2 = LPF2ordParam->b0;
    LPF2ordParam->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    LPF2ordParam->a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
    LPF2ordParam->delay_element1 = 0.0f;
    LPF2ordParam->delay_element2 = 0.0f;
}

/**
 * @brief  初始化二阶低通滤波器参数
 * @note   调用前需设置LPF2ordParam_t中的采样率与截止频率
 * @param  LPF2ordParam：参数结构体变量
 * @param  sample_freq：采样频率
 * @param  cutoff_freq：截止频率
 * @retval 无
 */
void LPF2ord_Init(LPF2ordParam_t *LPF2ordParam, float sample_freq, float cutoff_freq)
{
    LPF2ordParam->sample_freq = sample_freq;
    LPF2ordParam->cutoff_freq = cutoff_freq;
    LPF2ord_SetParam(LPF2ordParam);
}

/**
 * @brief  二阶低通滤波器调用函数
 * @note   注意不同的采样序列要将延迟元素分开储存
 * @param  LPF2ordParam:参数结构体变量
 * @param  simple:当前时刻采样值
 * @retval 滤波值
 */
float LPF2ord(LPF2ordParam_t *LPF2ordParam, float simple)
{
    /*直接I型*/
    /*需要4个延迟单元，对内存占用较高*/
    // float ret = LPF2ordParam->b0 * simple + LPF2ordParam->b1 * LPF2ordParam->inv_n_1 +
    //        LPF2ordParam->b2 * LPF2ordParam->inv_n_2 - LPF2ordParam->a1 * LPF2ordParam->outv_n_1 -
    //        LPF2ordParam->a2 * LPF2ordParam->outv_n_2;
    // LPF2ordParam->inv_n_2 = LPF2ordParam->inv_n_1;
    // LPF2ordParam->inv_n_1 = simple;
    // LPF2ordParam->outv_n_2 = LPF2ordParam->outv_n_1;
    // LPF2ordParam->outv_n_1 = ret;
    // return ret;

    /*直接II型*/
    /*需要两个延迟单元*/
    float delay_element0 = simple - LPF2ordParam->a1 * LPF2ordParam->delay_element1 -
                           LPF2ordParam->a2 * LPF2ordParam->delay_element2;
    float ret = LPF2ordParam->b0 * delay_element0 +
                LPF2ordParam->b1 * LPF2ordParam->delay_element1 +
                LPF2ordParam->b2 * LPF2ordParam->delay_element2;
    LPF2ordParam->delay_element2 = LPF2ordParam->delay_element1;
    LPF2ordParam->delay_element1 = delay_element0;
    return ret;
}

/*窗口大小*/
#define win_sz 5
extern float reference_pressure;
/**
 * @brief  滑动窗口平均滤波
 * @note
 * @param  无
 * @retval 无
 */
float moving_average(float in)
{
    static uint8_t index = 0;
    static float win_buf[win_sz];
    static bool win_full_flag;
    float sum = 0;
    uint8_t i = 0;
    win_buf[index] = in;
    index = (index + 1) % win_sz;
    /*窗口第一次初始化完成*/
    if (!win_full_flag && index == 0)
    {
        win_full_flag = true;
        for (i = 0; i < win_sz; i++)
        {
            sum += win_buf[i];
        }
        reference_pressure = sum / win_sz;
        return sum / win_sz;
    }
    if (!win_full_flag)
        return in;
    for (i = 0; i < win_sz; i++)
    {
        sum += win_buf[i];
    }
    return sum / win_sz;
}

#if 0
/**
  * @brief  滑动窗口平均滤波（带限幅）
  * @note   
  * @param  无
  * @retval 无
  */
float moving_average(float in)
{
    static uint8_t index = 0;
    static float win_buf[win_sz];
    static bool win_full_flag;
    float sum = 0;
    uint8_t i = 0;
    if (!win_full_flag)
    {
        win_buf[index] = in;
        index = (index + 1) % win_sz;
        if (index == 0)
            win_full_flag = true;
        return in;
    }
    for (i = 0; i < win_sz; i++)
    {
        sum += win_buf[i];
    }
    /*限幅*/
    if (fabs(index = 0 ? (in - win_buf[index]) : (in - win_buf[win_sz - 1])) < M)
    {
        win_buf[index] = in;
        index = (index + 1) % win_sz;
    }
    return sum / win_sz;
}
#endif
