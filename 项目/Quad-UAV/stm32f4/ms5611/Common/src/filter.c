/**
 ******************************************************************************
 * @file    filter.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/03/02
 * @brief   �˲�����
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

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*һ�׵�ͨ�˲�������*/
LPF1ordParam_t LPF1ordParam;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  ����һ�׵�ͨ�˲����˲�����
 * @note
 * @param  LPF1ordParam�������ṹ�����;
 * @param  sample_freq������Ƶ��;
 * @param  cutoff_freq����ֹƵ��;
 * @retval ��
 */
static void LPF1ord_SetParam(LPF1ordParam_t *LPF1ordParam, uint16_t sample_freq, uint16_t cutoff_freq)
{
    LPF1ordParam->Tc = 1 / (2 * PI * cutoff_freq);
    LPF1ordParam->a = 1 / (1 + LPF1ordParam->Tc * sample_freq);
}

/**
 * @brief  ��ʼ��һ�׵�ͨ�˲�������
 * @note
 * @param  LPF1ordParam�������ṹ�����;
 * @param  sample_freq������Ƶ��;
 * @param  cutoff_freq����ֹƵ��;
 * @retval ��
 */
void LPF1ord_Init(LPF1ordParam_t *LPF1ordParam, uint16_t sample_freq, uint16_t cutoff_freq)
{
    LPF1ord_SetParam(LPF1ordParam, sample_freq, cutoff_freq);
}

/**
 * @brief  һ�׵�ͨ�˲�������
 * @note
 * @param  LPF1ordParam�������ṹ�����;
 * @param  invalue������ֵ
 * @param  out_value_����һʱ�����ֵ
 * @retval �˲�ֵ
 */
float LPF1ord(LPF1ordParam_t *LPF1ordParam, float invalue, float out_value_)
{
    return LPF1ordParam->a * invalue + (1 - LPF1ordParam->a) * out_value_;
}

/**
 * @brief  ���ö��׵�ͨ�˲����˲�����
 * @note
 * @param  LPF2ordParam�������ṹ�����
 * @retval ��
 */
static void LPF2ord_SetParam(LPF2ordParam_t *LPF2ordParam)
{
    float fr = LPF2ordParam->sample_freq / LPF2ordParam->cutoff_freq;
    float ohm = tanf(PI / fr);
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
 * @brief  ��ʼ�����׵�ͨ�˲�������
 * @note   ����ǰ������LPF2ordParam_t�еĲ��������ֹƵ��
 * @param  LPF2ordParam�������ṹ�����
 * @param  sample_freq������Ƶ��
 * @param  cutoff_freq����ֹƵ��
 * @retval ��
 */
void LPF2ord_Init(LPF2ordParam_t *LPF2ordParam, float sample_freq, float cutoff_freq)
{
    LPF2ordParam->sample_freq = sample_freq;
    LPF2ordParam->cutoff_freq = cutoff_freq;
    LPF2ord_SetParam(LPF2ordParam);
}

/**
 * @brief  ���׵�ͨ�˲������ú���
 * @note   ע�ⲻͬ�Ĳ�������Ҫ���ӳ�Ԫ�طֿ�����
 * @param  LPF2ordParam:�����ṹ�����
 * @param  simple:��ǰʱ�̲���ֵ
 * @retval �˲�ֵ
 */
float LPF2ord(LPF2ordParam_t *LPF2ordParam, float simple)
{
    /*ֱ��I��*/
    /*��Ҫ4���ӳٵ�Ԫ�����ڴ�ռ�ýϸ�*/
    // float ret = LPF2ordParam->b0 * simple + LPF2ordParam->b1 * LPF2ordParam->inv_n_1 +
    //        LPF2ordParam->b2 * LPF2ordParam->inv_n_2 - LPF2ordParam->a1 * LPF2ordParam->outv_n_1 -
    //        LPF2ordParam->a2 * LPF2ordParam->outv_n_2;
    // LPF2ordParam->inv_n_2 = LPF2ordParam->inv_n_1;
    // LPF2ordParam->inv_n_1 = simple;
    // LPF2ordParam->outv_n_2 = LPF2ordParam->outv_n_1;
    // LPF2ordParam->outv_n_1 = ret;
    // return ret;

    /*ֱ��II��*/
    /*��Ҫ�����ӳٵ�Ԫ*/
    float delay_element0 = simple - LPF2ordParam->a1 * LPF2ordParam->delay_element1 -
                           LPF2ordParam->a2 * LPF2ordParam->delay_element2;
    float ret = LPF2ordParam->b0 * delay_element0 +
                LPF2ordParam->b1 * LPF2ordParam->delay_element1 +
                LPF2ordParam->b2 * LPF2ordParam->delay_element2;
    LPF2ordParam->delay_element2 = LPF2ordParam->delay_element1;
    LPF2ordParam->delay_element1 = delay_element0;
    return ret;
}

/*���ڴ�С*/
#define win_sz 5
extern float reference_pressure;
/**
 * @brief  ��������ƽ���˲�
 * @note
 * @param  ��
 * @retval ��
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
    /*���ڵ�һ�γ�ʼ�����*/
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
  * @brief  ��������ƽ���˲������޷���
  * @note   
  * @param  ��
  * @retval ��
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
    /*�޷�*/
    if (fabs(index = 0 ? (in - win_buf[index]) : (in - win_buf[win_sz - 1])) < M)
    {
        win_buf[index] = in;
        index = (index + 1) % win_sz;
    }
    return sum / win_sz;
}
#endif
