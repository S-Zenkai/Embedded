/**
  ******************************************************************************
  * @file    com_data.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/21
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_DATA_H
#define __COM_DATA_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/**********************************RC����******************************************/
/*ң�������ݽṹ��(9ͨ��)*/
typedef struct
{
    uint16_t channel1;
    uint16_t channel2;
    uint16_t channel3;
    uint16_t channel4;
    uint16_t channel5;
    uint16_t channel6;
    uint16_t channel7;
    uint16_t channel8;
    uint16_t channel9;
} rc_data_raw_t;
/*ң��������(9ͨ��)*/
// extern __IO rc_data_t rc_data;

/*��һ��ӳ����ң�������ݽṹ��*/
typedef struct
{
    float rc_roll;/*ͨ��1*/
    float rc_pitch;/*ͨ��2*/
    float rc_throttle;/*ͨ��3*/
    float rc_yaw;/*ͨ��4*/
    uint8_t switch_A;/*����A*/
    uint8_t switch_B;/*����B*/
    uint8_t switch_C;/*����C*/
    uint8_t switch_D;/*����D*/
    uint8_t switch_E;/*����E*/
} rc_data_norm_t;

/*ң�����ݴ����ܽṹ��*/
typedef struct
{
    rc_data_raw_t raw;/*���յ���ԭʼ����*/
    rc_data_norm_t norm;/*��һ��ӳ��������*/
} rc_state_t;


/*��һ�����ң��������(9ͨ��)*/
// extern __IO rc_data_norm_t rc_data_norm;
/*ң�����ݴ����ܽṹ��*/
extern __IO rc_state_t rc_state;


/**********************************���п�������******************************************/
/*����ö��*/
typedef enum
{
    ROLL,  /*��ת*/
    PITCH, /*����*/
    YAW,    /*ƫ��*/
    AXIS_COUNT/*������*/
} Axis_e;

/*����״̬�ṹ��*/
typedef struct
{
    float angle;/*�Ƕ�*/
    float angle_target;/*�����Ƕ�*/
    float rate;/*���ٶ�*/
    float rate_target;/*�������ٶ�*/
} Axis_State_t;

/*���п���״̬�ṹ��*/
typedef struct
{
    Axis_State_t control_state[AXIS_COUNT];/*�������״̬*/
    /*����״̬*/
    float throttle;/*����*/
} Flight_Control_State_t;

/*���п���״̬�ṹ��*/
extern __IO Flight_Control_State_t flight_control_state;

#endif /* __FILE_H */
