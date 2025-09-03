/**
  ******************************************************************************
  * @file    com_data.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/21
  * @brief   全局数据
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_DATA_H
#define __COM_DATA_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "com_type.h"
#include "ringbuff.h"


/**********************************RC数据******************************************/

/*归一化后的遥控器数据(9通道)*/
// extern __IO rc_data_norm_t rc_data_norm;
/*遥控数据处理总结构体*/
extern __IO rc_state_t rc_state;

/**********************************数据记录*************************************/
extern ringbuff_t SD_W_RingBuffMgr;

/**********************************传感器任务相关*************************************/
/*原始imu数据*/
typedef struct
{
    Axis3i16_t acc;
    Axis3i16_t gyro;
} IMU_RawData_t;

/*校准后的imu数据*/
typedef struct
{
    Axis3f_t acc;
    Axis3f_t gyro;
} IMU_CalData_t;

/*imu滤波数据*/
typedef struct
{
    Axis3f_t acc;
    Axis3f_t gyro;
} IMU_FilterData_t;

/*imu原始数据*/
extern IMU_RawData_t imu_raw_data;
/*imu校准数据*/
extern IMU_CalData_t imu_cal_data;
/*imu滤波数据*/
extern IMU_FilterData_t imu_filter_data;



/**********************************飞行控制数据******************************************/
/*飞行控制状态结构体*/
extern __IO Flight_Control_State_t flight_control_state;

#endif /* __FILE_H */
