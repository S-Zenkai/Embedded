/**
  ******************************************************************************
  * @file    pro_common.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/04/03
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRO_COMMON_H
#define __PRO_COMMON_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
// #include "com_type.h"
#include "com_data.h"

/*限幅函数*/
#define LIMIT(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))


/*角度转弧度*/
#define PI 3.1415926f
#define ANGLE_TO_RADIAN (PI/180.0f)
/*弧度转角度*/
#define RADIAN_TO_ANGLE (180.0f/PI)



/**********************************DMA相关*************************************/

#if GPS_UBX_IDLE_INTERRUPT_ENABLE
/*UBX-DMA缓冲区大小*/
/*需根据实际ubx包大小修改*/
#define UBX_DMA_BUFF_SIZE 512
/*UBX-DMA双缓冲区*/
extern uint8_t ubx_dma_buff[2][UBX_DMA_BUFF_SIZE];
/*活动缓冲区*/
extern uint8_t *ubx_active_buff;
#endif

/********************************GPS/UBX相关**********************************/

/*GPS-UBX使能空闲中断接收*/
#define GPS_UBX_IDLE_INTERRUPT_ENABLE 0
/*GPS-UBX缓冲区大小*/
//#define UBX_BUFF_SIZE 1024
//extern uint8_t ubx_buff[UBX_BUFF_SIZE];
//extern ringbuff_t ubx_rb;


/*****************************************************************************/

// extern Axis3i16_t mpu6000_raw_acc;
// extern Axis3i16_t mpu6000_raw_gyro;
// extern Axis3f_t mpu6000_cal_acc;
// extern Axis3f_t mpu6000_cal_gyro;

/*****************************************************************************/
//extern u16_u8_union_t rc_data[9];
extern uint8_t safety_switch;


/***************************数据类型****************************************/

// /*三维向量结构体*/
// typedef union
// {
//     struct
//     {
//         float x;
//         float y;
//         float z;
//     };
//     float arr[3];
// } Vector3_f_u;

// /*四维向量结构体(可用于四元数)*/
// typedef union
// {
//     struct
//     {
//         float w;
//         float x;
//         float y;
//         float z;
//     };
//     float arr[4];
// } Vector4_f_u;

/*3*3矩阵类型*/
typedef float Matrix3x3_f[3][3];
/*******************************外部函数**************************************/
void Matrix3x3_f_mul_Vector3_f(const Matrix3x3_f matrix, const Axis3_f_u *vector, Axis3_f_u *result);
void Matrix3x3_f_transpose(const Matrix3x3_f matrix, Matrix3x3_f result);
void Vector3_f_cross_product(const Axis3_f_u *vector1, const Axis3_f_u *vector2, Axis3_f_u *result);
void Vector3_f_add(const Axis3_f_u *vector1, const Axis3_f_u *vector2, Axis3_f_u *result);

#endif /* __PRO_COMMON_H */
