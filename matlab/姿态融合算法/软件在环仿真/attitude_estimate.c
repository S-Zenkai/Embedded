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

/*姿态解算结构体*/
typedef struct
{
    Mahony_Param_t Mahony_Param;
    bool attitude_estimate_init_flag;/*姿态解算初始化标志*/
} Attitude_Estimate_t;

Attitude_Estimate_t att_est;



/**
  * @brief  检查加速度计数据是否正常
  * @note   
  * @param  无
  * @retval 是否正常,true为正常,false为异常
  */
static bool Check_Acc_Data(const Axis3_f_u acc)
{
    /*三轴加速度平方和*/
    float sqs = acc.x * acc.x + acc.y * acc.y + acc.z * acc.z;
    /*计算比例因子*/
    float factor = sqs / (9.8f * 9.8f);
    /*如果比例因子大于0.8且小于1.2，则认为加速度计数据正常*/
    return factor < 0.8f&&factor > 1.2f;
}

/**
  * @brief  检查磁力计数据是否正常
  * @note   
  * @param  无
  * @retval 是否正常,true为正常,false为异常
  */
static bool Check_Mag_Data(const Axis3_f_u mag)
{
    /*检查磁力计数据是否正常*/
    return mag.x != 0.0f || mag.y != 0.0f || mag.z != 0.0f;
}

/**
  * @brief  四元数转欧拉角(北东地-前右下)
  * @note   
  * @param  无
  * @retval 无
  */
static void Quaternion_To_Euler(const Mahony_Param_t* M_P,Euler_u* el)
{
    // /*滚转角(-180°~180°)*/
    // el->roll = atan2f(2.0f * (M_P->q0 * M_P->q1 + M_P->q2 * M_P->q3),1.0f - 2.0f * (M_P->q1 * M_P->q1 + M_P->q2 * M_P->q2));
    // /*俯仰角(-90°~90°)*/
    // float sinp = 2.0f * (M_P->q0 * M_P->q2 - M_P->q3 * M_P->q1);
    // /*防止因浮点误差导致sinp超出asinf定义域范围*/
    // if(sinp > 1.0f)
    // {
    //     sinp = 1.0f;
    // }
    // else if(sinp < -1.0f)
    // {
    //     sinp = -1.0f;
    // }
    // el->pitch = asinf(sinp);
    // /*偏航角(-180°~180°)*/
    // el->yaw = atan2f(2.0f * (M_P->q0 * M_P->q3 + M_P->q1 * M_P->q2),1.0f - 2.0f * (M_P->q2 * M_P->q2 + M_P->q3 * M_P->q3));

    /*通过方向余弦矩阵求取(机体到地理)*/
    el->roll = atan2f(M_P->DCM[2][1],M_P->DCM[2][2]);
    /*防止因浮点误差导致asinf定义域范围*/
    float sinp = M_P->DCM[2][0];
    if(M_P->DCM[2][0] > 1.0f)
    {
        sinp = 1.0f;
    }
    else if(M_P->DCM[2][0] < -1.0f)
    {
        sinp = -1.0f;
    }
    el->pitch = -asinf(sinp);
    el->yaw = atan2f(M_P->DCM[1][0],M_P->DCM[0][0]);
}

/**
  * @brief  姿态解算更新
  * @note   
  * @param  无
  * @retval 无
  */
void Attitude_Estimate_Update(Attitude_Estimate_t* att_est,Flight_Control_State_t* flgt_ctl)
{
    att_est->Mahony_Param.use_acc = false;
    att_est->Mahony_Param.use_mag = false;
    if(att_est->attitude_estimate_init_flag == false)
    {
        Mahony_Init(&att_est->Mahony_Param,flgt_ctl->acc,flgt_ctl->mag);
        att_est->attitude_estimate_init_flag = true;
        return;
    }
    if(Check_Acc_Data(flgt_ctl->acc) == true)
    {
        att_est->Mahony_Param.use_acc = true;
    }
    if(Check_Mag_Data(flgt_ctl->mag) == true)
    {
        att_est->Mahony_Param.use_mag = true;
    }
    Mahony_Update(&att_est->Mahony_Param,flgt_ctl->acc,flgt_ctl->mag,flgt_ctl->gyro);
    Quaternion_To_Euler(&att_est->Mahony_Param,&flgt_ctl->euler);
}


