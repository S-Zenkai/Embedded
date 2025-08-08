/**
  ******************************************************************************
  * @file    com_type.h
  * @author  
  * @version V1.0.0
  * @data    2025/06/19
  * @brief   通用类型定义
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

/*三轴16位数据*/
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

/*三轴64位数据*/
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


/*三轴浮点数据*/
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

/*MPU6000原始数据结构体*/
// typedef struct
// {
//     Axis3i16_union_t AccRaw;/*原始数据*/
//     Axis3i16_union_t GyroRaw;
// } MPU6000RawData_t;

/*MPU6000校准数据结构体*/
// typedef struct
// {
//     Axis3f_union_t Acc;/*经处理的数据*/
//     Axis3f_union_t Gyro;
// } MPU6000CalData_t;

/*16位数据联合体*/
/*cortex-m采用小端存储，即低地址存低字节，高地址存高字节*/
typedef union
{
    uint16_t data;
    uint8_t data_arr[2];
}u16_u8_union_t;


#endif /* __COM_TYPE_H */
