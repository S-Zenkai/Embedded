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
 * ʵ��ƽ̨:
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
 * @brief       pid���ݽṹ��
 * 
 */
typedef struct
{
    uint16_t kp, kd, ki;
    float xe; /*״̬���*/
    float sum_xe;/*״̬����ۻ�*/
    float xe_last;/*��һʱ��״̬���*/
    float y;/*���������*/
} PID_DataTypeDef;

/**
 * @brief       ����pid���ݽṹ��
 * 
 */
typedef struct
{
    PID_DataTypeDef OuterPID_DataStruct;/*�⻷PID*/
    PID_DataTypeDef InnerPID_DataStruct;/*�ڻ�PID*/
    float y;/*���������*/
} CascadePID_DataTypeDef;
/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void PID_Init(PID_DataTypeDef *PID_DataStruct, uint16_t kp, uint16_t kd, uint16_t ki);
void PID_Cal(PID_DataTypeDef *PID_DataStruct, float x, float xd);
void Cascade_PID(CascadePID_DataTypeDef *CascadePID_DataStruct, float Inner_x, float Outer_x, float Outer_xd);
/*------------------------------------test------------------------------------*/
#endif	/* __CAR_PID_H */
