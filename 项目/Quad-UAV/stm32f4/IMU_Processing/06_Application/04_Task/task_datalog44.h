/**
  ******************************************************************************
  * @file    task_datalog.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/08/05
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_DATALOG_H
#define __TASK_DATALOG_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include <stdbool.h>
// #include "ff.h"       // FatFs头文件
// #include "ringbuff.h" // 你的环形缓冲区头文件

/* 1. 定义日志数据包类型 */
typedef enum 
{
    LOG_PACKET_IMU   = 0x01,  // IMU数据
    LOG_PACKET_ATT   = 0x02,  // 姿态数据
    LOG_PACKET_GPS   = 0x03,  // GPS数据
    // ... 在此添加其他数据类型
} LogPacketType_e;

/* 2. 定义统一的数据包头部 */
// 使用#pragma pack(1)确保结构体是紧凑的，没有编译器插入的填充字节
// 这对于文件存储和跨平台解析至关重要
// #pragma pack(1)
typedef struct
{
    uint16_t sync;       // 同步头, e.g., 0xABCD
    uint8_t  type;       // 包类型
    uint8_t  length;     // 数据负载的长度
    uint32_t timestamp;  // 时间戳 (ms)
    uint8_t  checksum;   // 校验和
} LogPacketHeader_t;
// #pragma pack()


/* 3. 定义具体的数据负载结构体 */
// #pragma pack(1)
// IMU数据结构体
// typedef struct
// {
//     float acc_x, acc_y, acc_z;
//     float gyro_x, gyro_y, gyro_z;
// } ImuData_t;

// // 姿态数据结构体 (欧拉角)
// typedef struct
// {
//     float roll, pitch, yaw;
// } AttitudeData_t;

// // GPS数据结构体 (简化版)
// typedef struct
// {
//     int32_t latitude;   // 纬度, 乘以1e7
//     int32_t longitude;  // 经度, 乘以1e7
//     uint8_t fix_type;   // 定位状态
//     uint8_t satellites; // 卫星数量
// } GpsData_t;
// #pragma pack()

#endif /* __TASK_DATALOG_H */
