/**
  ******************************************************************************
  * @file    filter.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   ͨ���˲����㷨ʵ��
  ******************************************************************************
  * @attention
  *
  * ���ļ�����һ�׵�ͨ�˲�����ʵ�֡�
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
 * @brief ��ʼ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param dt ����ʱ�� (��)
 * @param cutoff_freq ��ֹƵ�� (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq)
{
    if (cutoff_freq <= 0.0f) {
        filter->alpha = 1.0f; // �����ֹƵ����Ч�����˲�
    } else {
        float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
        filter->alpha = dt / (dt + rc);
    }
    filter->last_output = 0.0f;
}

/**
 * @brief Ӧ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param input ��ǰ������ֵ
 * @return �˲�������ֵ
 */
float lpf_apply(LPF_t *filter, float input)
{
    // ��ʽ: output = last_output + alpha * (input - last_output)
    float output = filter->last_output + filter->alpha * (input - filter->last_output);
    filter->last_output = output;
    return output;
}