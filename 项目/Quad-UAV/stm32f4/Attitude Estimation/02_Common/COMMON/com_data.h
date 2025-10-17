/**
  ******************************************************************************
  * @file    com_data.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/21
  * @brief   全局数据
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
#include "com_type.h"
#include "ringbuff.h"
/**********************************通用数据类型******************************************/
/*三轴有符号16位数据*/
typedef union
{
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis_arr[3];
} Axis3_i16_u;

/*三轴无符号16位数据*/
typedef union
{
    struct
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
    };
    uint16_t axis_arr[3];
} Axis3_u16_u;

/*三轴有符号32位数据*/
typedef union
{
    struct
    {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    int32_t axis_arr[3];
} Axis3_i32_u;

/*三轴无符号32位数据*/
typedef union
{
    struct
    {
        uint32_t x;
        uint32_t y;
        uint32_t z;
    };
    uint32_t axis_arr[3];
} Axis3_u32_u;


/*三轴有符号64位数据*/
typedef union
{
    struct
    {
        int64_t x;
        int64_t y;
        int64_t z;
    };
    int64_t axis_arr[3];
} Axis3_i64_u;

/*三轴无符号64位数据*/
typedef union
{
    struct
    {
        uint64_t x;
        uint64_t y;
        uint64_t z;
    };
    uint64_t axis_arr[3];
} Axis3_u64_u;

/*三轴浮点数数据*/
typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis_arr[3];
} Axis3_f_u;



/**********************************定时器相关******************************************/
extern uint32_t Tick;
/*主循环频率更新标志位*/
extern __IO FlagStatus Main_Loop_Update_Flag;

/*归一化后的遥控器数据(9通道)*/
// extern __IO rc_data_norm_t rc_data_norm;
/*遥控数据处理总结构体*/
extern __IO rc_state_t rc_state;

/**********************************数据记录*************************************/
extern ringbuff_t SD_W_RingBuffMgr;

/**********************************传感器任务相关*************************************/
/******************IMU相关************************/
/*原始imu数据*/
// typedef struct
// {
//     Axis3i16_t acc;
//     Axis3i16_t gyro;
// } IMU_RawData_t;
typedef struct
{
    Axis3_i16_u acc;
    Axis3_i16_u gyro;
} IMU_RawData_t;


/*校准后的imu数据*/
// typedef struct
// {
//     Axis3f_t acc;
//     Axis3f_t gyro;
// } IMU_CalData_t;
typedef struct
{
    Axis3_f_u acc;
    Axis3_f_u gyro;
} IMU_CalData_t;

/*imu滤波数据*/
// typedef struct
// {
//     Axis3f_t acc;
//     Axis3f_t gyro;
// } IMU_FilterData_t;
typedef struct
{
    Axis3_f_u acc;
    Axis3_f_u gyro;
} IMU_FilterData_t;

/*imu原始数据*/
extern IMU_RawData_t imu_raw_data;
/*imu校准数据*/
extern IMU_CalData_t imu_cal_data;
/*imu滤波数据*/
extern IMU_FilterData_t imu_filter_data;

/******************磁力计相关************************/
/*磁力计原始数据类型*/
typedef Axis3_i16_u MAG_RawData_t;
/*磁力计转换数据(高斯)*/
typedef Axis3_f_u MAG_GaussData_t;
/*磁力计校准数据类型*/
typedef Axis3_f_u MAG_CalData_t;
/*磁力计滤波数据类型*/
typedef Axis3_f_u MAG_FilterData_t;
/*磁力计原始数据*/
extern MAG_RawData_t mag_raw_data;
/*磁力计转换数据(高斯)*/
extern MAG_GaussData_t mag_gauss_data;
/*磁力计校准数据*/
extern MAG_CalData_t mag_cal_data;
/*磁力计滤波数据*/
extern MAG_FilterData_t mag_filter_data;





/**********************************飞行控制数据******************************************/
/*三轴枚举*/
typedef enum
{
    ROLL,      /*滚转*/
    PITCH,     /*俯仰*/
    YAW,       /*偏航*/
    AXIS_COUNT /*轴数量*/
} Axis_e;
/*状态结构体*/
typedef struct
{
    float angle;        /*角度*/
    float angle_target; /*期望角度*/
    float rate;         /*角速度*/
    float rate_target;  /*期望角速度*/
} Axis_State_t;

/*三轴欧拉角*/
typedef union
{
    struct
    {
        float roll;
        float pitch;
        float yaw;
    };
    float axis_arr[3];
} Euler_u;

/*飞行控制状态结构体*/
typedef struct
{
    Axis_State_t control_state[AXIS_COUNT]; /*三轴控制状态*/
    Axis3_f_u gyro;                 /*陀螺仪最终处理数据*/
    Axis3_f_u acc;                  /*加速度计最终处理数据*/
    Axis3_f_u mag;                  /*磁力计最终处理数据*/
    Euler_u euler;                /*欧拉角(北东地-前右下)*/
    /*其他状态*/
    float throttle; /*油门*/
} Flight_Control_State_t;
/*飞行控制状态结构体*/
extern __IO Flight_Control_State_t flgt_ctl;

#endif /* __FILE_H */
