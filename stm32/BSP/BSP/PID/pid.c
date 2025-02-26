/**
 * *****************************************************************************
 * @file        pid.c
 * @brief       
 * @author      
 * @date        2024-12-25
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
#include "pid.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/**
 * @brief       pid控制参数初始化
 * 
 * @param       PID_DataStruct pid数据结构体
 * @param       kp 比例系数
 * @param       ki 积分系数
 * @param       ki 微分系数
 */
void PID_Init(PID_DataTypeDef* PID_DataStruct, uint16_t kp, uint16_t ki, uint16_t kd)
{
    PID_DataStruct->kp = kp;
    PID_DataStruct->ki = ki;
    PID_DataStruct->kd = kd;
}

/**
 * @brief       pid控制函数，控制输出存储在PID_DataStruct->y
 *              使用积分限幅，幅值为2000(依据需求更改)
 * 
 * @param       PID_DataStruct pid数据结构体
 * @param       x 系统状态
 * @param       xd 期望状态
 */
void PID_Cal(PID_DataTypeDef* PID_DataStruct, float x, float xd)
{
    PID_DataStruct->xe_last = PID_DataStruct->xe;
    PID_DataStruct->xe = x - xd;
    PID_DataStruct->sum_xe += PID_DataStruct->xe;
    /*积分限幅*/
    if(PID_DataStruct->sum_xe > 2000)
    {
        PID_DataStruct->sum_xe = 2000;
    }
    PID_DataStruct->y = PID_DataStruct->kp * PID_DataStruct->xe +
                        PID_DataStruct->ki * PID_DataStruct->sum_xe +
                        PID_DataStruct->kd * (PID_DataStruct->xe - PID_DataStruct->xe_last);
}

/**
 * @brief       串级pid控制函数，外环输出作为内环期望信息，控制输出存储在CascadePID_DataStruct->y中。
 *              内环外环选择原则：
 *                              内环相比外环响应更快，可将要求快速响应的量设置为内环
 * 
 * @param       CascadePID_DataStruct 串级pid数据结构体
 * @param       Inner_x 内环状态
 * @param       Outer_x 外环状态
 * @param       Outer_xd 外环状态期望值
 */
void Cascade_PID(CascadePID_DataTypeDef* CascadePID_DataStruct,float Inner_x, float Outer_x, float Outer_xd)
{
    PID_Cal(&CascadePID_DataStruct->OuterPID_DataStruct, Outer_x, Outer_xd);
    PID_Cal(&CascadePID_DataStruct->InnerPID_DataStruct, Inner_x, CascadePID_DataStruct->OuterPID_DataStruct.y);
    CascadePID_DataStruct->y = CascadePID_DataStruct->InnerPID_DataStruct.y;
}
/*------------------------------------test------------------------------------*/

