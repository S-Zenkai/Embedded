/**
  ******************************************************************************
  * @file    imu_processing.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/08/08
  * @brief   IMU数据处理
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMU_PROCESSING_H__
#define __IMU_PROCESSING_H__
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdio.h>
#include "bsp_mpu6000.h"
#include "com_type.h"
#include "com_data.h"
/* Exported define ------------------------------------------------------------*/

#define DEBUG_IMU_PROCESSING_ENABLE
#ifdef DEBUG_IMU_PROCESSING_ENABLE
#define IMU_PROCESSING_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define IMU_PROCESSING_DEBUG(format, ...)
#endif

/* Exported types ------------------------------------------------------------*/

// /*原始imu数据*/
// typedef struct
// {
//     Axis3i16_t acc;
//     Axis3i16_t gyro;
// } IMU_RawData_t;

// /*校准后的imu数据*/
// typedef struct
// {
//     Axis3f_t acc;
//     Axis3f_t gyro;
// } IMU_CalData_t;

/*imu数据处理结构体*/
typedef struct
{
    /*陀螺仪数据处理部分*/
    Axis3f_t gyro_bias_mean;/*陀螺仪偏置均值*/
    Axis3i32_t gyro_square_sum;/*陀螺仪数据三轴平方和，使用int32_t防止溢出*/
    Axis3f_t gyro_bias_var;/*陀螺仪偏置方差*/
    uint32_t gyro_sample_count;/*采样数*/
    bool gyro_is_calibrated;/*是否校准*/
    /*加速度计数据处理部分*/
    float acc_scale_factor;/*加速度计缩放因子*/
    uint32_t acc_sample_count;/*采样数*/
    bool acc_is_calibrated;/*是否校准*/
} IMU_Processing_t;

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void IMU_Processing_Init(IMU_RawData_t *raw_data);
void IMU_Processing(IMU_RawData_t *imu_raw_data, IMU_CalData_t *imu_cal_data);

#endif /* __IMU_PROCESSING_H__ */
