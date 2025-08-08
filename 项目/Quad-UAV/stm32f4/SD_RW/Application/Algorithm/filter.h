/**
  ******************************************************************************
  * @file    filter.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   ͨ���˲����㷨ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  * ���ļ�����һ�׵�ͨ�˲�����ʵ�֡�
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
 * @brief һ�׵�ͨ�˲����ṹ��
 */
typedef struct {
    float alpha;          // �˲�ϵ��, alpha = dt / (dt + RC), ���� RC = 1 / (2*PI*cutoff_freq)
    float last_output;    // ��һ�ε��˲����
} LPF_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief ��ʼ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param dt ����ʱ�� (��)
 * @param cutoff_freq ��ֹƵ�� (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq);

/**
 * @brief Ӧ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param input ��ǰ������ֵ
 * @return �˲�������ֵ
 */
float lpf_apply(LPF_t *filter, float input);

#endif /* __FILTER_H */