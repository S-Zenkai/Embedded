/**
  ******************************************************************************
  * @file    attitude_estimate.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/09/30
  * @brief   姿态估计
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
#ifndef __ATTITUDE_ESTIMATE_H
#define __ATTITUDE_ESTIMATE_H
#include "pro_config.h"
#include "mahony.h"

/*使能加速度、地磁、角速度直接解算姿态(用于与数据融合算法解算的姿态进行对比)*/
#define ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE 1

#if ENABLE_ACC_MAG_GYRO_ATTITUDE_ESTIMATE
/*这段定义用于直接用加速度、角速度数据直接求取角度，与数据融合算法解算的姿态进行对比*/
typedef struct
{
    uint32_t tick;
    struct
    {
        float roll;
        float pitch;
        float yaw;
    };
} Euler_Test_t;
#endif

/*姿态解算结构体*/
typedef struct
{
    Mahony_Param_t Mahony_Param;
    bool attitude_estimate_init_flag;/*姿态解算初始化标志*/
} Attitude_Estimate_t;



extern Attitude_Estimate_t att_est;

void Attitude_Estimate_Update(Attitude_Estimate_t* att_est,__IO Flight_Control_State_t* flgt_ctl);

#endif /* __ATTITUDE_ESTIMATE_H */
