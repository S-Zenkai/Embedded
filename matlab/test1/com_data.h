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
// #include "stm32f4xx.h"

/**********************************RC����******************************************/
/*ң�������ݽṹ��(9ͨ��)*/
typedef struct
{
    unsigned short int channel1;
    unsigned short int channel2;
    unsigned short int channel3;
    unsigned short int channel4;
    unsigned short int channel5;
    unsigned short int channel6;
    unsigned short int channel7;
    unsigned short int channel8;
    unsigned short int channel9;
} rc_data_raw_t;
/*ң��������(9ͨ��)*/
// extern __IO rc_data_t rc_data;


/*��һ��ӳ����ң�������ݽṹ��*/
typedef struct
{
    double rc_roll;     /*ͨ��1*/
    double rc_pitch;    /*ͨ��2*/
    double rc_throttle; /*ͨ��3*/
    double rc_yaw;      /*ͨ��4*/
    unsigned char switch_A;  /*����A*/
    unsigned char switch_B;  /*����B*/
    unsigned char switch_C;  /*����C*/
    unsigned char switch_D;  /*����D*/
    unsigned char switch_E;  /*����E*/
} rc_data_norm_t;

/*ң�����ݴ����ܽṹ��*/
typedef struct
{
    rc_data_raw_t raw;   /*���յ���ԭʼ����*/
    rc_data_norm_t norm; /*��һ��ӳ��������*/
} rc_state_t;

/*��һ�����ң��������(9ͨ��)*/
// extern __IO rc_data_norm_t rc_data_norm;
/*ң�����ݴ����ܽṹ��*/
extern volatile rc_state_t rc_state;

/**********************************���п�������******************************************/
/*����ö��*/
typedef enum
{
    ROLL,      /*��ת*/
    PITCH,     /*����*/
    YAW,       /*ƫ��*/
    AXIS_COUNT /*������*/
} Axis_e;

/*����״̬�ṹ��*/
typedef struct
{
    double angle;        /*�Ƕ�*/
    double angle_target; /*�����Ƕ�*/
    double rate;         /*���ٶ�*/
    double rate_target;  /*�������ٶ�*/
} Axis_State_t;

/*���п���״̬�ṹ��*/
typedef struct
{
    Axis_State_t control_state[AXIS_COUNT]; /*�������״̬*/
    /*����״̬*/
    double throttle; /*����*/
} Flight_Control_State_t;

/*���п���״̬�ṹ��*/
extern volatile Flight_Control_State_t flight_control_state;

#endif /* __FILE_H */
