/**
 * *****************************************************************************
 * @file        kalman_filter.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2024-12-11
 * @version     
 * @copyright   
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "kalman_filter.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
#if 0
float gyro_bias = 0;
float angle = 0;
float dt = 0.005;//改了
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001;
float Q_gyro_bias = 0.003;
float R = 0.5;
void GitAngle(float Angle, float gyro)
{
    static float K[2][1] = {0, 0};
    static float angle_ = 0;
    static float gyro_bias_ = 0;
    static float P_[2][2] = {{0, 0}, {0, 0}};
    static float Z = 0;
    /*预测*/
    angle_ = angle + (gyro - gyro_bias) * dt;
    gyro_bias_ = gyro_bias;
    P_[0][0] = P[0][0] - (P[1][0] + P[0][1] - Q_angle) * dt;
    P_[0][1] = P[0][1] - P[1][1] * dt;
    P_[1][0] = P[1][0] - P[1][1] * dt;
    P_[1][1] = P[1][1] + Q_gyro_bias * dt;
    /*更新*/
    /*卡尔曼增益*/
    K[0][0] = P_[0][0] / (P_[0][0] + R);
    K[1][0] = P_[1][0] / (P_[0][0] + R);
    /*后验估计*/
    Z = Angle;/*这里对吗*/
    angle = angle_ + K[0][0]*(Z - angle_);
    gyro_bias = gyro_bias_ + K[1][0]*(Z - angle_);
    /*协方差*/
    P[0][0] = (1 - K[0][0]) * P_[0][0];
    P[0][1] = (1 - K[0][0]) * P_[0][1];
    P[1][0] = -K[1][0] * P_[0][0] + P_[1][0];
    P[1][1] = -K[1][0] * P_[0][1] + P_[1][1];
}
#elif 1
//卡尔曼滤波参数
float K1 =0.02; 
float angle, angle_dot;     
float Q_angle=0.001;    // 过程噪声的协方差
float Q_gyro=0.003;     //0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;      // 测量噪声的协方差 既测量偏差
float dt=0.005;         //                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/**
 * @description: 简易卡尔曼滤波   
 * @param {float} Accel 加速度计算得到的角度
 * @param {float} Gyro  角速度
 * @return {*}
 */
void Com_Filter_Kalman(float Accel,float Gyro)      
{
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
        
    Angle_err = Accel - angle;  //zk-先验估计
    
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
        
    angle   += K_0 * Angle_err;  //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    angle_dot   = Gyro - Q_bias;     //输出值(后验估计)的微分=角速度
}
#endif
/*------------------------------------test------------------------------------*/

