/**
  ******************************************************************************
  * @file    globle.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/04/03
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBLE_H
#define __GLOBLE_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/*******************************sbus���****************************************/
/*sbus�źŷ�Χ*/
#define SBUS_RANGE_MIN                  341.0f
#define SBUS_RANGR_MAX                  1706.0f
/*Ŀ��pwm��Χ*/
#define PWM_TARGRT_MIN                  1000.0f
#define PWM_TARGRT_MAX                  2000.0f
/*sbusЭ��8λͨ����*/
#define SBUS_INPUT_CHANNELS             25
/*������sbusЭ��11λͨ����*/
#define SBUS_PWM_CHANNELS               16
/*ң����֧��ͨ��*/
#define RC_PWM_CHANNELS                 9

extern uint8_t sbus_DF_TC;/*һ������֡(data frame)������ɱ�־*/
extern uint8_t sbus_buff[2][SBUS_INPUT_CHANNELS]; /*�໺����,��ֹ����ʱ���ݱ�����*/
extern uint8_t active_buff;/*���ջ�����ָ��*/
extern uint16_t pwm_buff[RC_PWM_CHANNELS];

#endif /* __GLOBLE_H */
