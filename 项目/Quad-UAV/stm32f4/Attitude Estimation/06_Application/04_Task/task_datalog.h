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
#include <stdio.h>
#include "com_type.h"
#include "ringbuff.h"

#define DEBUG_TASK_DATALOG1_ENABLE
#ifdef DEBUG_TASK_DATALOG1_ENABLE
#define TASK_DATALOG1_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define TASK_DATALOG1_DEBUG(format, ...)
#endif

#ifndef NULL
#define NULL 0
#endif



// 1. 定义每个数据包的唯一ID
#define LOG_ID_IMU_RAW      0x00/*IMU原始数据*/
#define LOG_ID_IMU_CAL      0x01/*IMU校准数据*/
#define LOG_ID_IMU_FILTER   0x02/*IMU滤波数据*/
#define LOG_ID_ATTITUDE     0x03/*姿态解算数据*/
#define LOG_ID_ACC_MAG_ATTITUDE 0x04/*加速度计、磁力计解算姿态数据*/
#define LOG_ID_GYRO_ATTITUDE 0x05/*陀螺仪解算姿态数据*/
#define LOG_ID_GPS          0x06/*GPS数据*/
#define LOG_ID_STATUS       0x07 /*系统状态信息*/
#define LOG_ID_MAG_RAW      0x08/*磁力计原始数据*/
#define LOG_ID_MAG_GAUSS    0x09/*磁力计高斯数据*/
#define LOG_ID_MAG_CAL      0x0A/*磁力计校准数据*/


// 2. 为每种数据类型定义独立的结构体
// 使用__attribute__((packed))确保没有内存对齐的填充字节，这对于跨平台解析很重要

/*原始imu数据*/
typedef struct __attribute__((packed)) {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
} log_imu_raw_t; // 12 bytes

/*校准、转换后的imu数据*/
typedef struct __attribute__((packed)) {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} log_imu_cal_t; // 24 bytes

/*滤波后的imu数据*/
typedef struct __attribute__((packed)) {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} log_imu_filter_t; // 24 bytes

/*姿态解算数据*/
typedef struct __attribute__((packed)) {
    float roll, pitch, yaw;
} log_attitude_t; // 12 bytes

/*加速度计、磁力计解算姿态数据*/
typedef struct __attribute__((packed)) {
    float roll, pitch, yaw;
} log_acc_mag_attitude_t; // 12 bytes

/*陀螺仪解算姿态数据*/
typedef struct __attribute__((packed)) {
    float roll, pitch, yaw;
} log_gyro_attitude_t; // 12 bytes

/*原始磁力计数据*/
typedef struct __attribute__((packed)) {
    int16_t mag_x, mag_y, mag_z;
} log_mag_raw_t; // 6 bytes

/*转换后的磁力计数据*/
typedef struct __attribute__((packed)) {
    float mag_x, mag_y, mag_z;
} log_mag_gauss_t; // 12 bytes

/*校准后的磁力计数据*/
typedef struct __attribute__((packed)) {
    float mag_x, mag_y, mag_z;
} log_mag_cal_t; // 12 bytes


typedef struct __attribute__((packed)) {
    double latitude;
    double longitude;
    float altitude;
    uint8_t sats_in_view;
} log_gps_t; // 21 bytes

typedef struct __attribute__((packed)) {
    uint8_t flight_mode;
    uint8_t is_armed;
} log_status_t; // 2 bytes


// 3. 定义统一的日志数据包结构 (Log Packet)
#define LOG_PACKET_HEADER_1 0xA5 // 包头同步字节1
#define LOG_PACKET_HEADER_2 0x5A // 包头同步字节2

typedef struct __attribute__((packed)) {
    uint8_t header1;      // 固定为 LOG_PACKET_HEADER_1
    uint8_t header2;      // 固定为 LOG_PACKET_HEADER_2
    uint8_t msg_id;       // 消息ID, e.g., LOG_ID_IMU
    uint8_t msg_len;      // payload的长度
    uint32_t timestamp;   // 时间戳 (ms or us)
    
    // 使用union来存储不同类型的数据，节省内存
    union {
        log_imu_raw_t imu_raw;
        log_imu_cal_t imu_cal;
        log_imu_filter_t imu_filter;
        log_attitude_t attitude;
        log_acc_mag_attitude_t acc_mag_attitude;
        log_gyro_attitude_t gyro_attitude;
        log_gps_t gps;
        log_status_t status;
        log_mag_raw_t mag_raw;
        log_mag_gauss_t mag_gauss;
        log_mag_cal_t mag_cal;
        
        // uint8_t raw_data[128]; // 定义一个足够大的原始缓冲区，确保union大小
    } payload;

    uint8_t checksum;     // 校验和
} log_packet_t;

void task_datalog(void);
void task_datalog_init(void);
void Push_Log_Packet_To_RingBuff(uint8_t msg_id, const void* payload_data, uint8_t payload_len, ringbuff_t* rb_mgr);
#endif /* __TASK_DATALOG_H */
