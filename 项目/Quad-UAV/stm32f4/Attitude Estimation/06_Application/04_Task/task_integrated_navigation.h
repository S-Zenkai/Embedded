/**
  ******************************************************************************
  * @file    task_integrated_navigation.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/09/30
  * @brief   组合导航任务
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
#ifndef __TASK_INTEGRATED_NAVIGATION_H
#define __TASK_INTEGRATED_NAVIGATION_H
#include "pro_config.h"
#include "attitude_estimate.h"

typedef struct
{
    uint32_t tick;
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float roll_acc;
    float pitch_acc;
    float yaw_acc;
    float yaw_mag;
} data_buf_t;

// typedef struct
// {
//     uint32_t tick;
//     int16_t acc_x;
//     int16_t acc_y;
//     int16_t acc_z;
//     int16_t gyro_x;
//     int16_t gyro_y;
//     int16_t gyro_z;
//     int16_t mag_x;
//     int16_t mag_y;
//     int16_t mag_z;
// } data_buf_i_t;

void Task_IntegratedNavigation_Init(void);
void Task_IntegratedNavigation(void);

#endif


