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
 * ʵ��ƽ̨:
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
 * @brief       pid���Ʋ�����ʼ��
 * 
 * @param       PID_DataStruct pid���ݽṹ��
 * @param       kp ����ϵ��
 * @param       ki ����ϵ��
 * @param       ki ΢��ϵ��
 */
void PID_Init(PID_DataTypeDef* PID_DataStruct, uint16_t kp, uint16_t ki, uint16_t kd)
{
    PID_DataStruct->kp = kp;
    PID_DataStruct->ki = ki;
    PID_DataStruct->kd = kd;
}

/**
 * @brief       pid���ƺ�������������洢��PID_DataStruct->y
 *              ʹ�û����޷�����ֵΪ2000(�����������)
 * 
 * @param       PID_DataStruct pid���ݽṹ��
 * @param       x ϵͳ״̬
 * @param       xd ����״̬
 */
void PID_Cal(PID_DataTypeDef* PID_DataStruct, float x, float xd)
{
    PID_DataStruct->xe_last = PID_DataStruct->xe;
    PID_DataStruct->xe = x - xd;
    PID_DataStruct->sum_xe += PID_DataStruct->xe;
    /*�����޷�*/
    if(PID_DataStruct->sum_xe > 2000)
    {
        PID_DataStruct->sum_xe = 2000;
    }
    PID_DataStruct->y = PID_DataStruct->kp * PID_DataStruct->xe +
                        PID_DataStruct->ki * PID_DataStruct->sum_xe +
                        PID_DataStruct->kd * (PID_DataStruct->xe - PID_DataStruct->xe_last);
}

/**
 * @brief       ����pid���ƺ������⻷�����Ϊ�ڻ�������Ϣ����������洢��CascadePID_DataStruct->y�С�
 *              �ڻ��⻷ѡ��ԭ��
 *                              �ڻ�����⻷��Ӧ���죬�ɽ�Ҫ�������Ӧ��������Ϊ�ڻ�
 * 
 * @param       CascadePID_DataStruct ����pid���ݽṹ��
 * @param       Inner_x �ڻ�״̬
 * @param       Outer_x �⻷״̬
 * @param       Outer_xd �⻷״̬����ֵ
 */
void Cascade_PID(CascadePID_DataTypeDef* CascadePID_DataStruct,float Inner_x, float Outer_x, float Outer_xd)
{
    PID_Cal(&CascadePID_DataStruct->OuterPID_DataStruct, Outer_x, Outer_xd);
    PID_Cal(&CascadePID_DataStruct->InnerPID_DataStruct, Inner_x, CascadePID_DataStruct->OuterPID_DataStruct.y);
    CascadePID_DataStruct->y = CascadePID_DataStruct->InnerPID_DataStruct.y;
}
/*------------------------------------test------------------------------------*/

