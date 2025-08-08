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
 * ʵ��ƽ̨:
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
float dt = 0.005;//����
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
    /*Ԥ��*/
    angle_ = angle + (gyro - gyro_bias) * dt;
    gyro_bias_ = gyro_bias;
    P_[0][0] = P[0][0] - (P[1][0] + P[0][1] - Q_angle) * dt;
    P_[0][1] = P[0][1] - P[1][1] * dt;
    P_[1][0] = P[1][0] - P[1][1] * dt;
    P_[1][1] = P[1][1] + Q_gyro_bias * dt;
    /*����*/
    /*����������*/
    K[0][0] = P_[0][0] / (P_[0][0] + R);
    K[1][0] = P_[1][0] / (P_[0][0] + R);
    /*�������*/
    Z = Angle;/*�������*/
    angle = angle_ + K[0][0]*(Z - angle_);
    gyro_bias = gyro_bias_ + K[1][0]*(Z - angle_);
    /*Э����*/
    P[0][0] = (1 - K[0][0]) * P_[0][0];
    P[0][1] = (1 - K[0][0]) * P_[0][1];
    P[1][0] = -K[1][0] * P_[0][0] + P_[1][0];
    P[1][1] = -K[1][0] * P_[0][1] + P_[1][1];
}
#elif 1
//�������˲�����
float K1 =0.02; 
float angle, angle_dot;     
float Q_angle=0.001;    // ����������Э����
float Q_gyro=0.003;     //0.03 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle=0.5;      // ����������Э���� �Ȳ���ƫ��
float dt=0.005;         //                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/**
 * @description: ���׿������˲�   
 * @param {float} Accel ���ٶȼ���õ��ĽǶ�
 * @param {float} Gyro  ���ٶ�
 * @return {*}
 */
void Com_Filter_Kalman(float Accel,float Gyro)      
{
    angle+=(Gyro - Q_bias) * dt; //�������
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
    PP[0][1] += Pdot[1] * dt;   // =����������Э����
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
        
    Angle_err = Accel - angle;  //zk-�������
    
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //����������Э����
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
        
    angle   += K_0 * Angle_err;  //�������
    Q_bias  += K_1 * Angle_err;  //�������
    angle_dot   = Gyro - Q_bias;     //���ֵ(�������)��΢��=���ٶ�
}
#endif
/*------------------------------------test------------------------------------*/

