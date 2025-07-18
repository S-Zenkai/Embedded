/**
  ******************************************************************************
  * @file    common.h
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
#ifndef __PRO_COMMON_H
#define __PRO_COMMON_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "com_type.h"
#include "stdbool.h"

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
extern uint8_t sbus_buff[SBUS_INPUT_CHANNELS]; /*�໺����,��ֹ����ʱ���ݱ�����*/
extern uint8_t active_buff;/*���ջ�����ָ��*/
extern uint16_t pwm_buff[RC_PWM_CHANNELS];

extern uint16_t rc_data[RC_PWM_CHANNELS];/*���RC����*/
extern bool rc_decode_done;

/**********************************DMA���*************************************/

#if GPS_UBX_IDLE_INTERRUPT_ENABLE
/*UBX-DMA��������С*/
/*�����ʵ��ubx����С�޸�*/
#define UBX_DMA_BUFF_SIZE 512
/*UBX-DMA˫������*/
extern uint8_t ubx_dma_buff[2][UBX_DMA_BUFF_SIZE];
/*�������*/
extern uint8_t *ubx_active_buff;
#endif

/********************************GPS/UBX���**********************************/

/*GPS-UBXʹ�ܿ����жϽ���*/
#define GPS_UBX_IDLE_INTERRUPT_ENABLE 0
/*GPS-UBX��������С*/
//#define UBX_BUFF_SIZE 1024
//extern uint8_t ubx_buff[UBX_BUFF_SIZE];
//extern ringbuff_t ubx_rb;


/*****************************************************************************/

extern Axis3i16_t mpu6000_raw_acc;
extern Axis3i16_t mpu6000_raw_gyro;
extern Axis3f_t mpu6000_cal_acc;
extern Axis3f_t mpu6000_cal_gyro;

/*****************************************************************************/
//extern u16_u8_union_t rc_data[9];
extern uint8_t safety_switch;

#endif /* __PRO_COMMON_H */
