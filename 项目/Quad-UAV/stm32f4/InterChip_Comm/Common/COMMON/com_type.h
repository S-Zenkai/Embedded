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

// typedef union
// {
//     Axis3i16_t axis_s;
//     int16_t axis_arr[3];
// } Axis3i16_union_t;

/*����64λ����*/
typedef struct
{
    int64_t x;
    int64_t y;
    int64_t z;
} Axis3i64_t;

// typedef union
// {
//     Axis3i64_t axis_s;
//     int64_t axis_arr[3];
// } Axis3i64_union_t; 


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


#endif /* __COM_TYPE_H */
