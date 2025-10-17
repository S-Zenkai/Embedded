/**
  ******************************************************************************
  * @file    mahony.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/09/24
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
#ifndef __MAHONY_H
#define __MAHONY_H
#include "pro_config.h"

/*mahony算法参数结构体*/
typedef struct
{
    float Kp; /*比例系数*/
    float Ki; /*积分系数*/
    float dt;/*采样间隔*/
    float integral_x;/*积分项*/
    float integral_y;/*积分项*/
    float integral_z;/*积分项*/
    float q0;
    float q1;
    float q2;
    float q3;
    float DCM[3][3]; /*方向余弦阵,机体坐标系到地理坐标系*/
    bool use_mag;    /*是否使用磁力计*/
    bool use_acc;    /*是否使用加速度计*/
    bool use_gps;    /*是否使用GPS修正偏航角*/
    bool attitude_estimate_init_flag;/*姿态解算初始化标志*/
} Mahony_Param_t;


/*函数声明*/
void Mahony_Init(Mahony_Param_t *Mhy_P, const Axis3_f_u acc, const Axis3_f_u mag);
void Mahony_Update(Mahony_Param_t *Mhy_P, const Axis3_f_u acc, const Axis3_f_u mag, const Axis3_f_u gyro);
float calculate_psi(float X_h, float Y_h);



#endif
