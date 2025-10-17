/**
  ******************************************************************************
  * @file    task_sensor.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/08/29
  * @brief   传感器任务
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_SENSOR_H
#define __TASK_SENSOR_H
/* Includes ------------------------------------------------------------------*/
#include "pro_config.h"

// typedef struct
// {
//     uint32_t tick;
//     float acc_x;
//     float acc_y;
//     float acc_z;
//     float gyro_x;
//     float gyro_y;
//     float gyro_z;
//     float mag_x;
//     float mag_y;
//     float mag_z;
// } data_buf_t;

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


void Task_Sensor_Init(void);
void Task_Sensor(void);
#endif /* __TASK_SENSOR_H */
