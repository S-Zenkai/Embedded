/**
  ******************************************************************************
  * @file    atti_control.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   姿态控制算法头文件
  ******************************************************************************
  * @attention
  *
  * 本文件包含串级PID姿态控制器的数据结构和函数原型。
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "com_data.h"
/* Exported define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/





/*PID控制器结构体，包括所有的参数和状态变量*/
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float sample_time;/*采样时间*/
    float last_measurement; /*上一次测量值*/
    float i_item;           /*积分项*/
    /*输出项*/
    float output_max;       /*输出限幅*/
    float output;           /*输出值*/
} PID_Controller_t;

/*形参上下文结构体*/
typedef struct
{
    __IO Flight_Control_State_t *flgt_ctl;
    PID_Controller_t *angle_pid;
    PID_Controller_t *ang_vel_pid;
    __IO rc_state_t *rc_state;
} atti_control_ctx_t;


// extern PID_Controller_t angle_pid[AXIS_COUNT];
// extern PID_Controller_t ang_vel_pid[AXIS_COUNT];
// extern atti_control_ctx_t atti_control_ctx;



/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void throttle_compensation(atti_control_ctx_t *atti_control_ctx);
void angle_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx);
void ang_vel_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx);
void atti_control_init(void);
void atti_control_update(void);

#endif /* __ATTITUDE_CONTROL_H */
