/**
  ******************************************************************************
  * @file    imu_processing.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/08/08
  * @brief   IMU���ݴ���
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

// /*ԭʼimu����*/
// typedef struct
// {
//     Axis3i16_t acc;
//     Axis3i16_t gyro;
// } IMU_RawData_t;

// /*У׼���imu����*/
// typedef struct
// {
//     Axis3f_t acc;
//     Axis3f_t gyro;
// } IMU_CalData_t;

/*imu���ݴ���ṹ��*/
typedef struct
{
    /*���������ݴ�����*/
    Axis3f_t gyro_bias_mean;/*������ƫ�þ�ֵ*/
    Axis3i32_t gyro_square_sum;/*��������������ƽ���ͣ�ʹ��int32_t��ֹ���*/
    Axis3f_t gyro_bias_var;/*������ƫ�÷���*/
    uint32_t gyro_sample_count;/*������*/
    bool gyro_is_calibrated;/*�Ƿ�У׼*/
    /*���ٶȼ����ݴ�����*/
    float acc_scale_factor;/*���ٶȼ���������*/
    uint32_t acc_sample_count;/*������*/
    bool acc_is_calibrated;/*�Ƿ�У׼*/
} IMU_Processing_t;

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void IMU_Processing_Init(IMU_RawData_t *raw_data);
void IMU_Processing(IMU_RawData_t *imu_raw_data, IMU_CalData_t *imu_cal_data);

#endif /* __IMU_PROCESSING_H__ */
