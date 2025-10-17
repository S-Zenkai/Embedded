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

typedef union
{
    Axis3i16_t axis_s;
    int16_t axis_arr[3];
} Axis3i16_union_t;




/*三轴16位数据*/
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

///*三轴32位数据*/
//typedef union
//{
//    struct
//    {
//        int32_t x;
//        int32_t y;
//        int32_t z;
//    };
//    int32_t axis_arr[3];
//} Axis3_i32_u;

/*三轴64位数据*/
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

/*三轴浮点数据*/
// typedef struct
// {
//     float x;
//     float y;
//     float z;
// } Axis3f_t;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis_arr[3];
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
} u16_u8_union_t;

/**********************************RC数据******************************************/
/*遥控器数据结构体(9通道)*/
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
/*遥控器数据(9通道)*/
// extern __IO rc_data_t rc_data;

/*归一化映射后的遥控器数据结构体*/
typedef struct
{
    float rc_roll;     /*通道1*/
    float rc_pitch;    /*通道2*/
    float rc_throttle; /*通道3*/
    float rc_yaw;      /*通道4*/
    uint8_t switch_A;  /*开关A*/
    uint8_t switch_B;  /*开关B*/
    uint8_t switch_C;  /*开关C*/
    uint8_t switch_D;  /*开关D*/
    uint8_t switch_E;  /*开关E*/
} rc_data_norm_t;

/*遥控数据处理总结构体*/
typedef struct
{
    rc_data_raw_t raw;   /*接收到的原始数据*/
    rc_data_norm_t norm; /*归一化映射后的数据*/
} rc_state_t;

/**********************************IMU数据******************************************/

/*IMU数据结构体*/
typedef struct
{
    Axis3i16_t acc;  /*加速度计*/
    Axis3i16_t gyro; /*陀螺仪*/
} imu_data_t;

/**********************************飞行控制数据******************************************/


/*三轴结构体*/
typedef struct
{
    float x;
    float y;
    float z;
} Axis_t;
/*三轴联合体*/
typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis_arr[3];
} Axis_union_t;





#endif /* __COM_TYPE_H */
