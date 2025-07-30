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
//#include "stm32f4xx.h"
#include "com_data.h"
#include "ProConfig.h"
#include "pro_common.h"
/* Exported define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
    double Kp;
    double Ki;
    double Kd;
    double sample_time;/*采样时间*/
    double last_measurement; /*上一次测量值*/
    double i_item;           /*积分项*/
    /*输出项*/
    double output_max;       /*输出限幅*/
    double output;           /*输出值*/
} PID_Controller_t;

/*形参上下文结构体*/
typedef struct
{
    Flight_Control_State_t *flight_control_state;
    PID_Controller_t *angle_pid;
    PID_Controller_t *ang_vel_pid;
    rc_state_t *rc_state;
} atti_control_ctx_t;


/*PID控制器结构体，包括所有的参数和状态变量*/


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
