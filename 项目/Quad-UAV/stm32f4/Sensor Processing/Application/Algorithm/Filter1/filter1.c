/**
  ******************************************************************************
  * @file    filter.c
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
#include "filter1.h"
#include "pro_common.h"
#include <math.h>

/**
  * @brief  һ�׵�ͨ�˲���ʼ��
  * @note   
  * @param  state: �˲���״̬�ṹ��
  * @param  cutoff_freq: ��ֹƵ��
  * @param  sample_freq: ����Ƶ��
  * @retval ��
  */
void LPF1_Init(LPF1_State_t *state, float cutoff_freq, float sample_freq)
{
    if (state == NULL|| cutoff_freq <= 0 || sample_freq <= 0)
    {
        return; // ��������
    }
    state->alpha = 1.0f / (1.0f + 2.0f * PI * cutoff_freq / sample_freq);
    state->is_first_filter = true;
}

/**
  * @brief  һ�׵�ͨ�˲�
  * @note   
  * @param  state: �˲���״̬�ṹ��ָ��
  * @param  input: ��������
  * @retval �˲��������
  */
float LPF1_Update(LPF1_State_t *state, float input)
{
    float output;
    if (state == NULL)
    {
        return input; // ��������
    }
    /*�״��˲�*/
    if (state->is_first_filter == true)
    {
        /*��ֹ��ʼʱ���������ɵ��˲��������*/
        state->prev_output = input;
        state->is_first_filter = false;
    }
    output = state->alpha * input + (1.0f - state->alpha) * state->prev_output;
    state->prev_output = output;
    return output;
}

/**
  * @brief  ����IIR��ͨ�˲�(ֱ��II��)��ʼ��
  * @note   
  * @param  state: �˲���״̬
  * @param  cutoff_freq: ��ֹƵ��
  * @param  sample_freq: ����Ƶ��
  * @retval ��
  */
void LPF2_Init(LPF2_State_t *state, float cutoff_freq, float sample_freq)
{
    if (state == NULL|| cutoff_freq <= 0 || sample_freq <= 0)
    {
        return; // ��������
    }
    /*�м����*/
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(PI / fr);/*Ƶ��Ԥ������������*/
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
  * @brief  ����IIR��ͨ�˲�(ֱ��II��)
  * @note   
  * @param  state: �˲���״̬�ṹ��ָ��
  * @param  input: ��������
  * @retval �˲��������
  */
float LPF2_Update(LPF2_State_t *state, float input)
{
    float output;
    if (state == NULL)
    {
        return input; // ��������
    }
    float delay_element0=input-state->beta1*state->delay_element1-state->beta2*state->delay_element2;
    /*������ֵͨ��*/
    /*��ֹ��ֵ���ȶ��ȵ��µ��˲�������������element0����Ϊinput�����������˲���*/
    if(!isfinite(delay_element0))
    {
        delay_element0 = input;
    }
    output = state->alpha0*delay_element0+state->alpha1*state->delay_element1+state->alpha2*state->delay_element2;
    state->delay_element2 = state->delay_element1;
    state->delay_element1 = delay_element0;
    return output;
}


