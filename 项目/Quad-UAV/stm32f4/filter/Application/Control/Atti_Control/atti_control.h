/**
  ******************************************************************************
  * @file    atti_control.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   ��̬�����㷨ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  * ���ļ���������PID��̬�����������ݽṹ�ͺ���ԭ�͡�
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


/*�β������Ľṹ��*/
typedef struct
{
    Flight_Control_State_t *flight_control_state;
    PID_Controller_t *angle_pid;
    PID_Controller_t *ang_vel_pid;
    rc_state_t *rc_state;
} atti_control_ctx_t;


/*PID�������ṹ�壬�������еĲ�����״̬����*/
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float sample_time;/*����ʱ��*/
    float last_measurement; /*��һ�β���ֵ*/
    float i_item;           /*������*/
    /*�����*/
    float output_max;       /*����޷�*/
    float output;           /*���ֵ*/
} PID_Controller_t;


extern PID_Controller_t angle_pid[AXIS_COUNT];
extern PID_Controller_t ang_vel_pid[AXIS_COUNT];
extern atti_control_ctx_t atti_control_ctx;



/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void throttle_compensation(atti_control_ctx_t *atti_control_ctx);
void angle_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx);
void ang_vel_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx);
void atti_control_init(void);
void atti_control_update(void);

#endif /* __ATTITUDE_CONTROL_H */
