/**
  ******************************************************************************
  * @file    com_data.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/21
  * @brief   ȫ������
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


/**********************************RC����******************************************/

/*��һ�����ң��������(9ͨ��)*/
// extern __IO rc_data_norm_t rc_data_norm;
/*ң�����ݴ����ܽṹ��*/
extern __IO rc_state_t rc_state;

/**********************************���ݼ�¼*************************************/
extern ringbuff_t SD_W_RingBuffMgr;

/**********************************�������������*************************************/
/*ԭʼimu����*/
typedef struct
{
    Axis3i16_t acc;
    Axis3i16_t gyro;
} IMU_RawData_t;

/*У׼���imu����*/
typedef struct
{
    Axis3f_t acc;
    Axis3f_t gyro;
} IMU_CalData_t;

/*imu�˲�����*/
typedef struct
{
    Axis3f_t acc;
    Axis3f_t gyro;
} IMU_FilterData_t;

/*imuԭʼ����*/
extern IMU_RawData_t imu_raw_data;
/*imuУ׼����*/
extern IMU_CalData_t imu_cal_data;
/*imu�˲�����*/
extern IMU_FilterData_t imu_filter_data;



/**********************************���п�������******************************************/
/*���п���״̬�ṹ��*/
extern __IO Flight_Control_State_t flight_control_state;

#endif /* __FILE_H */
