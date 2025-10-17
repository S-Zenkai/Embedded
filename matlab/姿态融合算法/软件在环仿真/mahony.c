/**
 ******************************************************************************
 * @file    mahony.c
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
#include "mahony.h"

/*mahony算法参数结构体*/
// Mahony_Param_t Mahony_Param;

/*旋转速度限制，当旋转速度大于此值时，失能积分项*/
#define ROTATION_SPEED_LIMIT 20.0f

/**
 * @brief  求取方向余弦阵(机体坐标系到地理坐标系)
 * @note
 * @param  Mahony_Param: mahony算法参数结构体
 * @retval 无
 */
void Get_DCM(Mahony_Param_t *Mahony_Param)
{
    float q1q1 = Mahony_Param->q1 * Mahony_Param->q1;
    float q2q2 = Mahony_Param->q2 * Mahony_Param->q2;
    float q3q3 = Mahony_Param->q3 * Mahony_Param->q3;
    float q0q1 = Mahony_Param->q0 * Mahony_Param->q1;
    float q0q2 = Mahony_Param->q0 * Mahony_Param->q2;
    float q0q3 = Mahony_Param->q0 * Mahony_Param->q3;
    float q1q2 = Mahony_Param->q1 * Mahony_Param->q2;
    float q1q3 = Mahony_Param->q1 * Mahony_Param->q3;
    float q2q3 = Mahony_Param->q2 * Mahony_Param->q3;
    Mahony_Param->DCM[0][0] = 1 - 2.0f * (q2q2 + q3q3);
    Mahony_Param->DCM[0][1] = 2.0f * (q1q2 - q0q3);
    Mahony_Param->DCM[0][2] = 2.0f * (q1q3 + q0q2);
    Mahony_Param->DCM[1][0] = 2.0f * (q1q2 + q0q3);
    Mahony_Param->DCM[1][1] = 1 - 2.0f * (q1q1 + q3q3);
    Mahony_Param->DCM[1][2] = 2.0f * (q2q3 - q0q1);
    Mahony_Param->DCM[2][0] = 2.0f * (q1q3 - q0q2);
    Mahony_Param->DCM[2][1] = 2.0f * (q2q3 + q0q1);
    Mahony_Param->DCM[2][2] = 1 - 2.0f * (q1q1 + q2q2);
}

/**
 * @brief  初始化
 * @note
 * @param  Mahony_Param: mahony算法参数结构体
 * @param  acc: 处理后的加速度计数据
 * @param  mag: 处理后的磁力计数据
 * @retval 无
 */
void Mahony_Init(Mahony_Param_t *Mahony_Param, const Axis3_f_u acc, const Axis3_f_u mag)
{
    Mahony_Param->Kp = 0.25f;
    Mahony_Param->Ki = 0.001f;
    /*初始欧拉角*/
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    /*通过加速度计数据计算初始俯仰角和横滚角*/
    roll = atan2f(-acc.y, -acc.z);
    pitch = atan2f(acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z));
    /*通过磁力计数据计算初始偏航角*/
    /*放平坐标轴后再计算偏航角*/
    float mag_x = mag.x * cosf(pitch) + mag.y * sinf(roll) * sinf(pitch) + mag.z * cosf(roll) * sinf(pitch);
    float mag_y = mag.y * cosf(roll) - mag.z * sinf(roll);
    yaw = atan2f(mag_y, mag_x);
    /*初始化四元数*/
    Mahony_Param->q0 = cosf(roll / 2.0f) * cosf(pitch / 2.0f) * cosf(yaw / 2.0f) + sinf(roll / 2.0f) * sinf(pitch / 2.0f) * sinf(yaw / 2.0f);
    Mahony_Param->q1 = sinf(roll / 2.0f) * cosf(pitch / 2.0f) * cosf(yaw / 2.0f) - cosf(roll / 2.0f) * sinf(pitch / 2.0f) * sinf(yaw / 2.0f);
    Mahony_Param->q2 = cosf(roll / 2.0f) * sinf(pitch / 2.0f) * cosf(yaw / 2.0f) + sinf(roll / 2.0f) * cosf(pitch / 2.0f) * sinf(yaw / 2.0f);
    Mahony_Param->q3 = cosf(roll / 2.0f) * cosf(pitch / 2.0f) * sinf(yaw / 2.0f) - sinf(roll / 2.0f) * sinf(pitch / 2.0f) * cosf(yaw / 2.0f);
    /*初始化方向余弦阵*/
    Get_DCM(Mahony_Param);

    Mahony_Param->use_gps = USE_GPS;
    Mahony_Param->dt = ATTITUDE_ESTIMATION_SAMPLE_TIME;
}

/**
 * @brief  mahony算法更新
 * @note
 * @param  无
 * @retval 无
 */
void Mahony_Update(Mahony_Param_t *Mahony_Param, const Axis3_f_u acc, const Axis3_f_u mag, const Axis3_f_u gyro)
{
    Axis3_f_u h_e;             /*磁力计数据,地理坐标系*/
    Axis3_f_u b_e;             /*磁力计参考矢量,地理坐标系*/
    Axis3_f_u b_b;             /*磁力计参考矢量,机体坐标系*/
    Axis3_f_u v_b;             /*加速度计参考矢量,机体坐标系*/
    Axis3_f_u e_b = {0, 0, 0}; /*参考向量与测量向量的误差变量,机体坐标系*/
    Axis3_f_u temp;            /*中间变量*/
    float DCM_T[3][3];         /*DCM的转置,地理坐标系到机体坐标系*/
    float norm;
    /*归一化后的磁力计数据*/
    Axis3_f_u mag_normalized;
    /*归一化后的加速度计数据*/
    Axis3_f_u acc_normalized;
    Axis3_f_u gyro_corrected; /*修正后的角速度*/
    if (Mahony_Param->use_gps)
    {
        /*未实现*/
        /*还未搞懂*/
    }
    /*磁力计修正*/
    /*磁力计数据范数*/
    norm = sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    /*磁力计修正*/
    if (Mahony_Param->use_mag && norm > 0.1f) /*这里使用0.1是为了防止除零以及norm过小造成的浮点数溢出*/
    {
        /*归一化磁力计数据*/
        mag_normalized.x = mag.x / norm;
        mag_normalized.y = mag.y / norm;
        mag_normalized.z = mag.z / norm;
        /*将磁力计数据转换到地理坐标系*/
        Matrix3x3_f_mul_Vector3_f(Mahony_Param->DCM, &mag_normalized, &h_e);
        /*构造参考矢量*/
        b_e.x = sqrtf(h_e.x * h_e.x + h_e.y * h_e.y);
        b_e.y = 0.0f;
        b_e.z = h_e.z;
        /*参考矢量转换到机体坐标系*/
        Matrix3x3_f_transpose(Mahony_Param->DCM, DCM_T);
        Matrix3x3_f_mul_Vector3_f(DCM_T, &b_e, &b_b);
        /*计算误差变量，通过叉乘衡量误差，叉乘具有反对称性，这个要注意*/
        Vector3_f_cross_product(&mag_normalized, &b_b, &temp);
        /*累计误差*/
        Vector3_f_add(&e_b, &temp, &e_b);
    }
    /*acc修正*/
    norm = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    if (Mahony_Param->use_acc && norm > 0.1f)
    {
        /*归一化加速度计数据*/
        acc_normalized.x = acc.x / norm;
        acc_normalized.y = acc.y / norm;
        acc_normalized.z = acc.z / norm;
        /*构造参考矢量*/
        /*v_e={0,0,1}*/
        /*参考矢量转换到机体坐标系*/
        v_b.x = Mahony_Param->DCM[2][0];
        v_b.y = Mahony_Param->DCM[2][1];
        v_b.z = Mahony_Param->DCM[2][2];
        /*计算误差变量，通过叉乘衡量误差，叉乘具有反对称性，这个要注意*/
        Vector3_f_cross_product(&acc_normalized, &v_b, &temp);
        /*累计误差*/
        Vector3_f_add(&e_b, &temp, &e_b);
    }
    /*使用pi控制对角速度进行修正*/
    /*积分项*/
    norm = sqrtf(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
    /*当飞机高速旋转时，加速度计测量的不再主要是重力，此时重力参考矢量不可用，不在使用加速度计误差积分*/
    if (Mahony_Param->Ki > 0 && norm < ROTATION_SPEED_LIMIT * ANGLE_TO_RADIAN)
    {
        Mahony_Param->integral_x += e_b.x * Mahony_Param->Ki * Mahony_Param->dt;
        Mahony_Param->integral_y += e_b.y * Mahony_Param->Ki * Mahony_Param->dt;
        Mahony_Param->integral_z += e_b.z * Mahony_Param->Ki * Mahony_Param->dt;
    }
    /*修正角速度*/
    gyro_corrected.x = gyro.x + Mahony_Param->Kp * e_b.x + Mahony_Param->integral_x;
    gyro_corrected.y = gyro.y + Mahony_Param->Kp * e_b.y + Mahony_Param->integral_y;
    gyro_corrected.z = gyro.z + Mahony_Param->Kp * e_b.z + Mahony_Param->integral_z;
    /*求取四元数(欧拉法)*/
    Mahony_Param->q0 += 0.5f * (-Mahony_Param->q1 * gyro_corrected.x - Mahony_Param->q2 * gyro_corrected.y - Mahony_Param->q3 * gyro_corrected.z) * Mahony_Param->dt;
    Mahony_Param->q1 += 0.5f * (Mahony_Param->q0 * gyro_corrected.x + Mahony_Param->q2 * gyro_corrected.z - Mahony_Param->q3 * gyro_corrected.y) * Mahony_Param->dt;
    Mahony_Param->q2 += 0.5f * (Mahony_Param->q0 * gyro_corrected.y - Mahony_Param->q1 * gyro_corrected.z + Mahony_Param->q3 * gyro_corrected.x) * Mahony_Param->dt;
    Mahony_Param->q3 += 0.5f * (Mahony_Param->q0 * gyro_corrected.z + Mahony_Param->q1 * gyro_corrected.y - Mahony_Param->q2 * gyro_corrected.x) * Mahony_Param->dt;
    /*归一化四元数*/
    norm = sqrtf(Mahony_Param->q0 * Mahony_Param->q0 + Mahony_Param->q1 * Mahony_Param->q1 + Mahony_Param->q2 * Mahony_Param->q2 + Mahony_Param->q3 * Mahony_Param->q3);
    Mahony_Param->q0 /= norm;
    Mahony_Param->q1 /= norm;
    Mahony_Param->q2 /= norm;
    Mahony_Param->q3 /= norm;
    /*更新方向余弦阵*/
    Get_DCM(Mahony_Param);
}
