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
void Get_DCM(Mahony_Param_t *Mhy_P)
{
    float q1q1 = Mhy_P->q1 * Mhy_P->q1;
    float q2q2 = Mhy_P->q2 * Mhy_P->q2;
    float q3q3 = Mhy_P->q3 * Mhy_P->q3;
    float q0q1 = Mhy_P->q0 * Mhy_P->q1;
    float q0q2 = Mhy_P->q0 * Mhy_P->q2;
    float q0q3 = Mhy_P->q0 * Mhy_P->q3;
    float q1q2 = Mhy_P->q1 * Mhy_P->q2;
    float q1q3 = Mhy_P->q1 * Mhy_P->q3;
    float q2q3 = Mhy_P->q2 * Mhy_P->q3;
    Mhy_P->DCM[0][0] = 1 - 2.0f * (q2q2 + q3q3);
    Mhy_P->DCM[0][1] = 2.0f * (q1q2 - q0q3);
    Mhy_P->DCM[0][2] = 2.0f * (q1q3 + q0q2);
    Mhy_P->DCM[1][0] = 2.0f * (q1q2 + q0q3);
    Mhy_P->DCM[1][1] = 1 - 2.0f * (q1q1 + q3q3);
    Mhy_P->DCM[1][2] = 2.0f * (q2q3 - q0q1);
    Mhy_P->DCM[2][0] = 2.0f * (q1q3 - q0q2);
    Mhy_P->DCM[2][1] = 2.0f * (q2q3 + q0q1);
    Mhy_P->DCM[2][2] = 1 - 2.0f * (q1q1 + q2q2);
    // if (isnan(Mhy_P->DCM[0][0]) || isnan(Mhy_P->DCM[0][1]) || isnan(Mhy_P->DCM[0][2]) ||
    //     isnan(Mhy_P->DCM[1][0]) || isnan(Mhy_P->DCM[1][1]) || isnan(Mhy_P->DCM[1][2]) ||
    //     isnan(Mhy_P->DCM[2][0]) || isnan(Mhy_P->DCM[2][1]) || isnan(Mhy_P->DCM[2][2]))
    // {
    //     __NOP(); /*在这里打断点*/
    // }
}

float calculate_psi(float X_h, float Y_h)
{
    // C语言的 atan 函数返回的是弧度，需要转换为角度
    // 转换公式: angle_degrees = angle_radians * 180.0 / M_PI
    float angle_in_degrees = atan(Y_h / X_h) * 180.0f / PI;

    // 根据 X_h 和 Y_h 的值应用不同的公式
    if (X_h < 0)
    {
        return 180.0f - angle_in_degrees;
    }
    else if (X_h > 0 && Y_h < 0)
    {
        return -angle_in_degrees;
    }
    else if (X_h > 0 && Y_h > 0)
    {
        return 360.0f - angle_in_degrees;
    }
    else if (X_h == 0 && Y_h < 0)
    {
        return 90.0;
    }
    else if (X_h == 0 && Y_h > 0)
    {
        return 270.0;
    }
    else
    {
        // 公式中未定义的其他情况 (例如 X_h > 0, Y_h = 0 或 X_h = 0, Y_h = 0)
        // 在此返回 0.0 作为默认值
        return 0.0;
    }
}


/**
 * @brief  初始化
 * @note
 * @param  Mahony_Param: mahony算法参数结构体
 * @param  acc: 处理后的加速度计数据
 * @param  mag: 处理后的磁力计数据
 * @retval 无
 */
void Mahony_Init(Mahony_Param_t *Mhy_P, const Axis3_f_u acc, const Axis3_f_u mag)
{
    Mhy_P->Kp = MAHONY_KP;
    Mhy_P->Ki = MAHONY_KI;
    /*初始欧拉角*/
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    /*通过加速度计数据计算初始俯仰角和横滚角*/
    roll = atan2f(-acc.y, -acc.z);
    pitch = atan2f(acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z));
    /*通过磁力计数据计算初始偏航角*/
    /*放平坐标轴后再计算偏航角*/
    float mag_x = mag.x * cosf(pitch) +
                  mag.y * sinf(roll) * sinf(pitch) +
                  mag.z * cosf(roll) * sinf(pitch);
    float mag_y = mag.y * cosf(roll) - mag.z * sinf(roll);
    yaw = calculate_psi(mag_x, mag_y)* ANGLE_TO_RADIAN;
    /*初始化四元数*/
    Mhy_P->q0 = cosf(roll / 2.0f) * cosf(pitch / 2.0f) * cosf(yaw / 2.0f) + sinf(roll / 2.0f) * sinf(pitch / 2.0f) * sinf(yaw / 2.0f);
    Mhy_P->q1 = sinf(roll / 2.0f) * cosf(pitch / 2.0f) * cosf(yaw / 2.0f) - cosf(roll / 2.0f) * sinf(pitch / 2.0f) * sinf(yaw / 2.0f);
    Mhy_P->q2 = cosf(roll / 2.0f) * sinf(pitch / 2.0f) * cosf(yaw / 2.0f) + sinf(roll / 2.0f) * cosf(pitch / 2.0f) * sinf(yaw / 2.0f);
    Mhy_P->q3 = cosf(roll / 2.0f) * cosf(pitch / 2.0f) * sinf(yaw / 2.0f) - sinf(roll / 2.0f) * sinf(pitch / 2.0f) * cosf(yaw / 2.0f);
    /*初始化方向余弦阵*/
    Get_DCM(Mhy_P);

    Mhy_P->use_gps = USE_GPS;
    Mhy_P->dt = ATTITUDE_ESTIMATION_SAMPLE_TIME;
}

extern int32_t num_test;

/**
 * @brief  mahony算法更新
 * @note
 * @param  无
 * @retval 无
 */
void Mahony_Update(Mahony_Param_t *Mhy_P, const Axis3_f_u acc, const Axis3_f_u mag, const Axis3_f_u gyro)
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
    if (Mhy_P->use_gps)
    {
        /*未实现*/
        /*还未搞懂*/
    }
    /*磁力计修正*/
    /*磁力计数据范数*/
    norm = sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    // if (norm == 0.0f || isnan(norm))
    // {
    //     __NOP(); /*在这里打断点*/
    // }
    /*磁力计修正*/
    if (Mhy_P->use_mag && norm > 0.1f) /*这里使用0.1是为了防止除零以及norm过小造成的浮点数溢出*/
    {
        /*归一化磁力计数据*/
        mag_normalized.x = mag.x / norm;
        mag_normalized.y = mag.y / norm;
        mag_normalized.z = mag.z / norm;
        /*将磁力计数据转换到地理坐标系*/
        Matrix3x3_f_mul_Vector3_f((const float(*)[3])Mhy_P->DCM, &mag_normalized, &h_e);
        // if (isnan(h_e.x) || isnan(h_e.y) || isnan(h_e.z))
        // {
        //     __NOP(); /*在这里打断点*/
        // }
        /*构造参考矢量*/
        b_e.x = sqrtf(h_e.x * h_e.x + h_e.y * h_e.y);
        b_e.y = 0.0f;
        b_e.z = h_e.z;
        /*参考矢量转换到机体坐标系*/
        Matrix3x3_f_transpose((const float(*)[3])Mhy_P->DCM, DCM_T);
        Matrix3x3_f_mul_Vector3_f((const float(*)[3])DCM_T, &b_e, &b_b);
        /*计算误差变量，通过叉乘衡量误差，叉乘具有反对称性，这个要注意*/
        Vector3_f_cross_product(&mag_normalized, &b_b, &temp);
        /*累计误差*/
        Vector3_f_add(&e_b, &temp, &e_b);
    }
    /*acc修正*/
    norm = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    if (Mhy_P->use_acc && norm > 0.1f)
    {
        /*归一化加速度计数据*/
        acc_normalized.x = acc.x / norm;
        acc_normalized.y = acc.y / norm;
        acc_normalized.z = acc.z / norm;
        /*构造参考矢量*/
        /*v_e={0,0,1}*/
        /*参考矢量转换到机体坐标系*/
        v_b.x = -Mhy_P->DCM[2][0];
        v_b.y = -Mhy_P->DCM[2][1];
        v_b.z = -Mhy_P->DCM[2][2];
        /*计算误差变量，通过叉乘衡量误差，叉乘具有反对称性，这个要注意*/
        Vector3_f_cross_product(&acc_normalized, &v_b, &temp);
        /*累计误差*/
        Vector3_f_add(&e_b, &temp, &e_b);
    }
    /*使用pi控制对角速度进行修正*/
    /*积分项*/
    norm = sqrtf(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
    /*当飞机高速旋转时，加速度计测量的不再主要是重力，此时重力参考矢量不可用，不在使用加速度计误差积分*/
    if (Mhy_P->Ki > 0 && norm < ROTATION_SPEED_LIMIT * ANGLE_TO_RADIAN)
    {
        Mhy_P->integral_x += e_b.x * Mhy_P->Ki * Mhy_P->dt;
        Mhy_P->integral_y += e_b.y * Mhy_P->Ki * Mhy_P->dt;
        Mhy_P->integral_z += e_b.z * Mhy_P->Ki * Mhy_P->dt;
    }
    /*修正角速度*/
    gyro_corrected.x = gyro.x + Mhy_P->Kp * e_b.x + Mhy_P->integral_x;
    gyro_corrected.y = gyro.y + Mhy_P->Kp * e_b.y + Mhy_P->integral_y;
    gyro_corrected.z = gyro.z + Mhy_P->Kp * e_b.z + Mhy_P->integral_z;
    // if (Mhy_P->q0 > 100.0f || Mhy_P->q1 > 100.0f || Mhy_P->q2 > 100.0f || Mhy_P->q3 > 100.0f ||
    //     Mhy_P->q0 < -100.0f || Mhy_P->q1 < -100.0f || Mhy_P->q2 < -100.0f || Mhy_P->q3 < -100.0f)
    // {
    //     __NOP(); /*在这里打断点*/
    // }
    /*求取四元数(欧拉法)*/
    Mhy_P->q0 += 0.5f * (-Mhy_P->q1 * gyro_corrected.x - Mhy_P->q2 * gyro_corrected.y - Mhy_P->q3 * gyro_corrected.z) * Mhy_P->dt;
    Mhy_P->q1 += 0.5f * (Mhy_P->q0 * gyro_corrected.x + Mhy_P->q2 * gyro_corrected.z - Mhy_P->q3 * gyro_corrected.y) * Mhy_P->dt;
    Mhy_P->q2 += 0.5f * (Mhy_P->q0 * gyro_corrected.y - Mhy_P->q1 * gyro_corrected.z + Mhy_P->q3 * gyro_corrected.x) * Mhy_P->dt;
    Mhy_P->q3 += 0.5f * (Mhy_P->q0 * gyro_corrected.z + Mhy_P->q1 * gyro_corrected.y - Mhy_P->q2 * gyro_corrected.x) * Mhy_P->dt;
    // if (Mhy_P->q0 > 100.0f || Mhy_P->q1 > 100.0f || Mhy_P->q2 > 100.0f || Mhy_P->q3 > 100.0f ||
    //     Mhy_P->q0 < -100.0f || Mhy_P->q1 < -100.0f || Mhy_P->q2 < -100.0f || Mhy_P->q3 < -100.0f)
    // {
    //     __NOP(); /*在这里打断点*/
    // }
    /*归一化四元数*/
    norm = sqrtf(Mhy_P->q0 * Mhy_P->q0 + Mhy_P->q1 * Mhy_P->q1 + Mhy_P->q2 * Mhy_P->q2 + Mhy_P->q3 * Mhy_P->q3);
    Mhy_P->q0 /= norm;
    Mhy_P->q1 /= norm;
    Mhy_P->q2 /= norm;
    Mhy_P->q3 /= norm;
    // if (num_test == 4)
    // {
    //     __NOP(); /*在这里打断点*/
    // }
    // if (isnan(Mhy_P->q0) || isnan(Mhy_P->q1) || isnan(Mhy_P->q2) || isnan(Mhy_P->q3))
    // {
    //     __NOP(); /*在这里打断点*/
    // }
    /*更新方向余弦阵*/
    Get_DCM(Mhy_P);
}
