/**
  ******************************************************************************
  * @file    com_type.h
  * @author  
  * @version V1.0.0
  * @data    2025/06/19
  * @brief   ͨ�����Ͷ���
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
#ifndef __COM_TYPE_H
#define __COM_TYPE_H
#include "stm32f4xx.h"

/*����16λ����*/
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Axis3i16_t;

typedef union
{
    Axis3i16_t axis_s;
    int16_t axis_arr[3];
} Axis3i16_union_t;

/*����16λ����*/
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} Axis3i32_t;

typedef union
{
    Axis3i32_t axis_s;
    int32_t axis_arr[3];
} Axis3i32_union_t;

/*����64λ����*/
typedef struct
{
    int64_t x;
    int64_t y;
    int64_t z;
} Axis3i64_t;

typedef union
{
    Axis3i64_t axis_s;
    int64_t axis_arr[3];
} Axis3i64_union_t; 


/*���ḡ������*/
typedef struct
{
    float x;
    float y;
    float z;
} Axis3f_t;

// typedef union
// {
//     Axis3f_t axis_s;
//     float axis_arr[3];
// } Axis3f_union_t;

/*MPU6000ԭʼ���ݽṹ��*/
// typedef struct
// {
//     Axis3i16_union_t AccRaw;/*ԭʼ����*/
//     Axis3i16_union_t GyroRaw;
// } MPU6000RawData_t;

/*MPU6000У׼���ݽṹ��*/
// typedef struct
// {
//     Axis3f_union_t Acc;/*�����������*/
//     Axis3f_union_t Gyro;
// } MPU6000CalData_t;

/*16λ����������*/
/*cortex-m����С�˴洢�����͵�ַ����ֽڣ��ߵ�ַ����ֽ�*/
typedef union
{
    uint16_t data;
    uint8_t data_arr[2];
}u16_u8_union_t;

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


/**********************************IMU����******************************************/



/*IMU���ݽṹ��*/
typedef struct
{
    Axis3i16_t acc;/*���ٶȼ�*/
    Axis3i16_t gyro;/*������*/
} imu_data_t;

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

#endif /* __COM_TYPE_H */
