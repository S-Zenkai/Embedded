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
    float rc_roll;/*通道1*/
    float rc_pitch;/*通道2*/
    float rc_throttle;/*通道3*/
    float rc_yaw;/*通道4*/
    uint8_t switch_A;/*开关A*/
    uint8_t switch_B;/*开关B*/
    uint8_t switch_C;/*开关C*/
    uint8_t switch_D;/*开关D*/
    uint8_t switch_E;/*开关E*/
} rc_data_norm_t;

/*遥控数据处理总结构体*/
typedef struct
{
    rc_data_raw_t raw;/*接收到的原始数据*/
    rc_data_norm_t norm;/*归一化映射后的数据*/
} rc_state_t;


/*归一化后的遥控器数据(9通道)*/
// extern __IO rc_data_norm_t rc_data_norm;
/*遥控数据处理总结构体*/
extern __IO rc_state_t rc_state;


/**********************************飞行控制数据******************************************/
/*三轴枚举*/
typedef enum
{
    ROLL,  /*滚转*/
    PITCH, /*俯仰*/
    YAW,    /*偏航*/
    AXIS_COUNT/*轴数量*/
} Axis_e;

/*三轴状态结构体*/
typedef struct
{
    float angle;/*角度*/
    float angle_target;/*期望角度*/
    float rate;/*角速度*/
    float rate_target;/*期望角速度*/
} Axis_State_t;

/*飞行控制状态结构体*/
typedef struct
{
    Axis_State_t control_state[AXIS_COUNT];/*三轴控制状态*/
    /*其他状态*/
    float throttle;/*油门*/
} Flight_Control_State_t;

/*飞行控制状态结构体*/
extern __IO Flight_Control_State_t flight_control_state;

#endif /* __FILE_H */
