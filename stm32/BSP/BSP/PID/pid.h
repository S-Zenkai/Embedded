/**
 * *****************************************************************************
 * @file        car_pid.h
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

#ifndef __PID_H 
#define __PID_H 
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
/**
 * @brief       pid数据结构体
 * 
 */
typedef struct
{
    uint16_t kp, kd, ki;
    float xe; /*状态误差*/
    float sum_xe;/*状态误差累积*/
    float xe_last;/*上一时刻状态误差*/
    float y;/*控制器输出*/
} PID_DataTypeDef;

/**
 * @brief       串级pid数据结构体
 * 
 */
typedef struct
{
    PID_DataTypeDef OuterPID_DataStruct;/*外环PID*/
    PID_DataTypeDef InnerPID_DataStruct;/*内环PID*/
    float y;/*控制器输出*/
} CascadePID_DataTypeDef;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void PID_Init(PID_DataTypeDef *PID_DataStruct, uint16_t kp, uint16_t kd, uint16_t ki);
void PID_Cal(PID_DataTypeDef *PID_DataStruct, float x, float xd);
void Cascade_PID(CascadePID_DataTypeDef *CascadePID_DataStruct, float Inner_x, float Outer_x, float Outer_xd);
/*------------------------------------test------------------------------------*/
#endif	/* __CAR_PID_H */
